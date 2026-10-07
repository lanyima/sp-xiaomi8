"""Low-rate, near-neutral automatic white balance for the Xiaomi 8 V4L2 camera.

The IFE consumes /data/wb_gains.txt on its per-frame update path.  CamX normally
supplies AWB statistics; this lightweight companion supplies the missing feedback
without treating vegetation or other saturated objects as grey-world references.
"""
import os
import sys
import time

import numpy as np

from msgq.visionipc import VisionIpcClient, VisionStreamType


GAINS_PATH = "/data/wb_gains.txt"
DEFAULT_GAINS = np.array([1.0, 2.0246, 1.9471], dtype=np.float32)  # G, B, R
SAMPLE_X = np.linspace(96, 1823, 72, dtype=np.int32)
SAMPLE_Y = np.linspace(72, 1007, 48, dtype=np.int32)
MIN_NEUTRAL_SAMPLES = 12
UPDATE_PERIOD_SECONDS = 5.0
MAX_STEP_FRACTION = 0.01


def read_gains() -> np.ndarray:
  try:
    values = np.array([float(v) for v in open(GAINS_PATH).read().split()], dtype=np.float32)
    if values.shape == (3,) and np.all(np.isfinite(values)) and np.all((values > 0.3) & (values < 5.0)):
      return values
  except (OSError, ValueError):
    pass
  return DEFAULT_GAINS.copy()


def write_gains(gains: np.ndarray) -> None:
  # Atomic replacement keeps IFE's periodic reader from seeing a partial line.
  temporary = GAINS_PATH + ".awbd.new"
  with open(temporary, "w") as f:
    f.write("%.4f %.4f %.4f\n" % tuple(gains))
  os.replace(temporary, GAINS_PATH)


def neutral_ratios(frame) -> tuple[float, float, int] | None:
  """Return robust R/G, B/G and neutral sample count from a sparse NV12 grid."""
  y_plane = np.frombuffer(frame.data[:frame.uv_offset], dtype=np.uint8).reshape((-1, frame.stride))
  uv_height = ((frame.height // 2) + 15) // 16 * 16
  uv_plane = np.frombuffer(frame.data[frame.uv_offset:frame.uv_offset + frame.stride * uv_height], dtype=np.uint8).reshape((-1, frame.stride))
  yy, xx = np.meshgrid(SAMPLE_Y[SAMPLE_Y < frame.height], SAMPLE_X[SAMPLE_X < frame.width], indexing="ij")
  y = y_plane[yy, xx].astype(np.float32)
  uv_x = (xx // 2) * 2
  u = uv_plane[yy // 2, uv_x].astype(np.float32) - 128.0
  v = uv_plane[yy // 2, uv_x + 1].astype(np.float32) - 128.0
  r = y + 1.13983 * v
  g = y - 0.39465 * u - 0.58060 * v
  b = y + 2.03211 * u
  rgb = np.stack((r, g, b), axis=-1)
  hi, lo = rgb.max(axis=-1), rgb.min(axis=-1)
  # Reject clipping, shadows, and chromatic subjects. Do not require a large
  # proportion of the scene: road scenes may contain only a small neutral patch
  # (lane paint, concrete, cloud), and requiring 8% made indoor AWB inert.
  keep = (hi > 45.0) & (hi < 220.0) & ((hi - lo) < 0.22 * hi)
  if int(keep.sum()) < MIN_NEUTRAL_SAMPLES:
    return None
  selected = rgb[keep]
  mean = np.median(selected, axis=0)
  if mean[1] < 8.0:
    return None
  return float(mean[0] / mean[1]), float(mean[2] / mean[1]), int(keep.sum())


def main() -> None:
  gains = read_gains()
  history: list[tuple[float, float, int]] = []
  last_write = 0.0
  while True:
    # camerad owns the VisionIPC server and can be restarted independently of
    # this companion. Reconnect instead of silently spinning on a dead client;
    # retain no pre-restart colour samples across that boundary.
    client = VisionIpcClient("camerad", VisionStreamType.VISION_STREAM_ROAD, True)
    while not client.connect(False):
      time.sleep(0.5)
    history.clear()
    empty_frames = 0
    while True:
      frame = client.recv()
      if frame is None:
        empty_frames += 1
        if empty_frames >= 30:
          break
        time.sleep(0.05)
        continue
      empty_frames = 0
      result = neutral_ratios(frame)
      if result is None:
        continue
      rg, bg, samples = result
      history.append(result)
      history = history[-15:]
    # A previous 8% update once per second visibly pumped colour in motion. Use
    # a temporal-consensus gate plus a hard 1% step at most every five seconds.
    # This makes AWB a slow CCT adaptation, not a per-frame colour controller.
      if len(history) < 15 or time.monotonic() - last_write < UPDATE_PERIOD_SECONDS:
        continue
      ratios = np.asarray([(x[0], x[1]) for x in history], dtype=np.float32)
      median = np.median(ratios, axis=0)
      if np.max(np.abs(ratios - median)) > 0.05:
        continue
      rg, bg = map(float, median)
      alpha = 0.01
      candidate = gains.copy()
      candidate[2] *= rg ** (-alpha)  # R gain
      candidate[1] *= bg ** (-alpha)  # B gain
      candidate[1:] = np.clip(candidate[1:], 0.75, 3.5)
      if np.max(np.abs(candidate - gains)) < 0.002:
        continue
      gains = candidate
      write_gains(gains)
      last_write = time.monotonic()
      print("awbd neutral=%.3f/%.3f samples=%d gains=%.3f,%.3f,%.3f" %
            (rg, bg, samples, gains[0], gains[1], gains[2]), flush=True)


if __name__ == "__main__":
  main()
