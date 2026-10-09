#!/usr/bin/env python3
"""Read-only sample of the six CAN messages required by Geely Panda safety."""
import time

from openpilot.cereal import messaging

REQUIRED = ((0, 0x0E0), (0, 0x150), (0, 0x084), (0, 0x122), (0, 0x125), (2, 0x1A1))


def main() -> None:
  counts = dict.fromkeys(REQUIRED, 0)
  acc_samples = []
  sm = messaging.SubMaster(["can"])
  deadline = time.monotonic() + 10.0
  while time.monotonic() < deadline:
    sm.update(100)
    for packet in sm["can"]:
      key = (packet.src, packet.address)
      if key in counts:
        counts[key] += 1
      if key == (2, 0x1A1) and len(acc_samples) < 16:
        data = bytes(packet.dat)
        crc = 0xFF
        for byte in data[:-1]:
          crc ^= byte
          for _ in range(8):
            crc = ((crc << 1) ^ 0x2F) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
        acc_samples.append((data.hex(), data[5] & 0xF, data[7], crc ^ 0xFF))
  for (bus, address), count in counts.items():
    print(f"bus={bus} addr=0x{address:03X} count={count} hz={count / 10.0:.1f}")
  for data, counter, actual, expected in acc_samples:
    print(f"0x1A1 data={data} counter={counter} crc={actual:02X} expected={expected:02X}")


if __name__ == "__main__":
  main()
