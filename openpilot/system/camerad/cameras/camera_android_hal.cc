/*
 * Android Camera HAL backend for camerad
 *
 * Reads NV12 frames from shared memory written by hal_capture_daemon
 * (a bionic executable using Camera2 NDK) and sends them to VisionIPC.
 *
 * Single rear camera only — Xiaomi Mi 8 HAL does not support concurrent
 * front+rear capture. Road frames are mirrored to wide and driver streams
 * for compatibility with openpilot subsystems.
 *
 * Architecture:
 *   hal_capture_daemon (bionic, cam=0) → SHM → camerad (glibc) → VisionIPC
 */

#include "system/camerad/cameras/camera_common.h"

#include <fcntl.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <signal.h>
#include <setjmp.h>
#include <algorithm>
#include <atomic>
#include <cassert>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <string>
#include <thread>

#include "common/clutil.h"
#include "common/params.h"
#include "common/swaglog.h"
#include "common/util.h"

// ====================== Shared Memory Protocol ======================
// Must match hal_capture_daemon.c

#define SHM_PATH "/data/android_root/tmp/hal_camera"
#define SHM_HEADER_SIZE 128

typedef struct __attribute__((packed)) {
  volatile uint64_t seq;             // offset 0: seqlock: odd = writing, even = valid
  volatile uint64_t timestamp_sof;   // offset 8
  volatile uint64_t timestamp_eof;   // offset 16
  volatile uint32_t frame_id;        // offset 24
  volatile uint32_t width;           // offset 28
  volatile uint32_t height;          // offset 32
  volatile uint32_t stride;          // offset 36
  volatile uint32_t frame_size;      // offset 40: total NV12 bytes
  volatile uint32_t status;          // offset 44: 0=init, 1=streaming, 2=error, 3=shutdown
  volatile uint32_t fps;             // offset 48: fps * 10
  volatile uint32_t dropped;         // offset 52
  uint8_t _pad[128 - 56];           // offset 56: pad to 128
} HalCameraHeader;

ExitHandler do_exit;

// ====================== SIGBUS Recovery ======================
// hal_capture_daemon crash or SHM file truncation can cause SIGBUS
// when camerad reads from the mmap'd region. We use siglongjmp to
// recover and reconnect to the SHM file instead of dying.

static sigjmp_buf g_sigbus_jmp;
static volatile sig_atomic_t g_in_frame_read = 0;

static void sigbus_handler(int sig) {
  if (g_in_frame_read) {
    siglongjmp(g_sigbus_jmp, 1);
  }
  // Not in protected region — restore default and re-raise
  signal(SIGBUS, SIG_DFL);
  raise(SIGBUS);
}

// Publish a FrameData cereal message
#define PUBLISH_FRAME(pm, topic, init_fn, fid, sof, eof) do { \
  MessageBuilder _msg; \
  auto _f = _msg.initEvent().init_fn(); \
  _f.setFrameId(fid); \
  _f.setRequestId(fid); \
  _f.setTimestampEof(eof); \
  _f.setTimestampSof(sof); \
  _f.setIntegLines(0); \
  _f.setGain(1.0f); \
  _f.setHighConversionGain(false); \
  _f.setMeasuredGreyFraction(0.0f); \
  _f.setTargetGreyFraction(0.3f); \
  _f.setProcessingTime(0.0f); \
  _f.setExposureValPercent(50.0f); \
  _f.setSensor(cereal::FrameData::ImageSensor::UNKNOWN); \
  (pm).send(topic, _msg); \
} while(0)

// ====================== Main Camera Thread ======================

void camerad_thread() {
  LOG("=== camerad: Android Camera HAL backend (rear camera only) ===");

  // xiaomi8 IMX363 HAL3 native: 1920x1080 tightly packed. We pad/restride
  // to QCOM Venus NV12 layout for comma3X 1928x1208 (logical) so:
  //   1) official warp_1928x1208_tinygrad.pkl applies as-is (no FileNotFound)
  //   2) Adreno EGL accepts the buffer (EGL_BAD_ALLOC otherwise — the GPU
  //      requires Y stride aligned to 128B, total size per VENUS_BUFFER_SIZE).
  //
  // get_nv12_info(1928, 1208) reports stride=2048, y_h=1216, uv_h=608,
  // total=4804608.  We hard-code these to avoid pulling in nv12_info.h.
  const int src_width  = 1920;
  const int src_height = 1080;
  const int src_stride = src_width;
  const int src_uv_offset = src_stride * src_height;
  const int src_nv12_size = src_stride * src_height * 3 / 2;

  const int cam_width  = 1928;          // published to VisionIPC + topic
  const int cam_height = 1208;
  const int stride     = 2048;          // QCOM Venus Y stride for 1928 (align 128)
  const int y_h        = 1216;          // align(1208, 32)
  const int uv_offset  = stride * y_h;  // 2490368
  const int nv12_size  = 4804608;       // VENUS_BUFFER_SIZE NV12 1928x1208
  const int shm_size   = SHM_HEADER_SIZE + src_nv12_size;   // SHM remains 1920x1080

  // Install SIGBUS handler for SHM recovery
  struct sigaction sa = {}, old_sa = {};
  sa.sa_handler = sigbus_handler;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = 0;
  sigaction(SIGBUS, &sa, &old_sa);

  // VisionIPC setup (persistent across SHM reconnects)
  // xiaomi8: CL unused after VisionIpcServer ctor change
  // cl_device_id device_id = cl_get_device_id(CL_DEVICE_TYPE_DEFAULT);
  // cl_context ctx = CL_CHECK_ERR(clCreateContext(NULL, 1, &device_id, NULL, NULL, &err));

  const bool publish_wide = (getenv("NO_WIDE") == nullptr);

  VisionIpcServer vipc_server("camerad");  // xiaomi8: upstream removed CL args
  vipc_server.create_buffers_with_sizes(VISION_STREAM_ROAD, VIPC_BUFFER_COUNT,
                                         cam_width, cam_height, nv12_size, stride, uv_offset);
  if (publish_wide) {
    vipc_server.create_buffers_with_sizes(VISION_STREAM_WIDE_ROAD, VIPC_BUFFER_COUNT,
                                           cam_width, cam_height, nv12_size, stride, uv_offset);
  }
  vipc_server.start_listener();
  LOG("VisionIPC: %dx%d (road%s)", cam_width, cam_height, publish_wide ? "+wide" : " only");

  PubMaster pm(publish_wide ? std::vector<const char *>{"roadCameraState", "wideRoadCameraState"}
                            : std::vector<const char *>{"roadCameraState"});
  uint32_t cnt = 0;

  // ======================== SHM reconnect loop ========================
  for (int shm_attempt = 0; !do_exit; shm_attempt++) {
    if (shm_attempt > 0) {
      int delay = std::min(2 + shm_attempt, 10);
      LOG("SHM reconnect (attempt %d), waiting %ds...", shm_attempt, delay);
      for (int d = 0; d < delay && !do_exit; d++) sleep(1);
      if (do_exit) break;
    }

    // Wait for SHM to appear AND reach correct size (up to 180s)
    LOG("Waiting for SHM file %s (size=%d, up to 180s)...", SHM_PATH, shm_size);
    for (int w = 0; !do_exit && w < 360; w++) {
      struct stat st;
      if (stat(SHM_PATH, &st) == 0 && st.st_size >= shm_size) break;
      usleep(500000);
      if (w > 0 && w % 20 == 0) {
        struct stat s2;
        off_t cur = (stat(SHM_PATH, &s2) == 0) ? s2.st_size : -1;
        LOG("  still waiting... (%ds, file_size=%lld, need=%d)", w / 2, (long long)cur, shm_size);
      }
      if (w == 359) { LOGE("Timeout waiting for %s (size)", SHM_PATH); }
    }
    if (do_exit) break;

    int shm_fd = open(SHM_PATH, O_RDONLY);
    if (shm_fd < 0) { LOGE("open(%s): %s", SHM_PATH, strerror(errno)); continue; }

    struct stat shm_stat;
    if (fstat(shm_fd, &shm_stat) < 0 || shm_stat.st_size < shm_size) {
      LOGE("SHM file too small: %lld < %d", (long long)shm_stat.st_size, shm_size);
      close(shm_fd);
      continue;
    }

    void *shm_ptr = mmap(NULL, shm_size, PROT_READ, MAP_SHARED, shm_fd, 0);
    if (shm_ptr == MAP_FAILED) { LOGE("mmap: %s", strerror(errno)); close(shm_fd); continue; }

    const HalCameraHeader *hdr = (const HalCameraHeader *)shm_ptr;
    const uint8_t *frame_data = (const uint8_t *)shm_ptr + SHM_HEADER_SIZE;

    // Wait for streaming
    LOG("Waiting for HAL daemon streaming...");
    for (int w = 0; !do_exit && hdr->status != 1; w++) {
      usleep(500000);
      if (w > 240) { LOGE("Timeout (status=%u)", hdr->status); break; }
      if (w > 0 && w % 20 == 0) LOG("  streaming wait (%ds, st=%u)", w / 2, hdr->status);
    }
    if (do_exit) { munmap(shm_ptr, shm_size); close(shm_fd); break; }
    if (hdr->status != 1) { munmap(shm_ptr, shm_size); close(shm_fd); continue; }

    LOG("Streaming: %ux%u stride=%u fps=%.1f", hdr->width, hdr->height, hdr->stride, hdr->fps / 10.0f);

    // SIGBUS recovery point — if SIGBUS fires during frame read, we land here
    if (sigsetjmp(g_sigbus_jmp, 1) != 0) {
      g_in_frame_read = 0;
      LOGW("SIGBUS recovery: SHM invalid, will reconnect...");
      munmap(shm_ptr, shm_size);
      close(shm_fd);
      continue;  // back to SHM reconnect loop
    }

    // ======================== Frame read loop ========================
    uint32_t last_fid = UINT32_MAX;
    uint64_t last_log = nanos_since_boot();
    int stale = 0;

    LOG("Main loop started (attempt %d)", shm_attempt);

    while (!do_exit) {
      uint64_t seq1 = hdr->seq;
      __sync_synchronize();

      if (seq1 & 1) { usleep(100); continue; }  // writer active

      if (hdr->frame_id == last_fid) {
        if (++stale > 10000) {
          LOGW("No frames ~1s (fid=%u st=%u)", hdr->frame_id, hdr->status);
          stale = 0;
          if (hdr->status >= 2) { LOGE("Daemon stopped (st=%u)", hdr->status); break; }
          // Check if SHM was recreated (inode changed) — hal3_direct restart
          struct stat st;
          if (stat(SHM_PATH, &st) == 0) {
            struct stat fd_st;
            if (fstat(shm_fd, &fd_st) == 0 && st.st_ino != fd_st.st_ino) {
              LOGW("SHM inode changed (%llu -> %llu), reconnecting...",
                   (unsigned long long)fd_st.st_ino, (unsigned long long)st.st_ino);
              break;
            }
          }
        }
        // Adaptive sleep: short spin initially, then back off
        if (stale < 50) {
          usleep(100);   // 100us spin for first 5ms — catch frame quickly
        } else {
          usleep(5000);  // 5ms backoff — save CPU while waiting
        }
        continue;
      }
      stale = 0;

      // Periodically verify SHM file still valid (every ~500 frames / ~25s)
      if (cnt % 500 == 0 && cnt > 0) {
        struct stat st;
        if (fstat(shm_fd, &st) < 0 || st.st_size < shm_size) {
          LOGW("SHM file changed (nlink=%lld size=%lld), reconnecting...",
               (long long)st.st_nlink, (long long)st.st_size);
          break;
        }
      }

      // xiaomi8 Day 26 fix: snapshot frame_id/sof/eof BEFORE memcpy.
      // hal3_direct seqlock increments seq per frame, so seq change after
      // memcpy is normal (next frame started), NOT torn read. Use snapshot
      // values throughout to avoid black-flash / frame skipping.
      uint32_t cur_fid = hdr->frame_id;
      uint64_t sof = hdr->timestamp_sof;
      uint64_t eof = hdr->timestamp_eof;

      VisionBuf *rb = vipc_server.get_buffer(VISION_STREAM_ROAD);
      if (!rb || !rb->addr) { usleep(1000); continue; }

      // Protected memcpy — SIGBUS recovery enabled.
      // Pad src 1920x1080 NV12 (stride=1920) into dst 1928x1208 NV12
      // (stride=2048 QCOM Venus layout). dst was pre-zeroed by
      // VisionIpcServer.create_buffers; we touch only the active region
      // so right pad cols (1920..2048) and bottom pad rows (1080..1216 Y,
      // 540..608 UV) stay zero. The modeld warp normalises to image
      // coordinates so the inactive region collapses to the edge.
      g_in_frame_read = 1;
      uint8_t *dst = (uint8_t *)rb->addr;
      const uint8_t *src_y  = frame_data;
      const uint8_t *src_uv = frame_data + src_uv_offset;
      uint8_t *dst_y  = dst;
      uint8_t *dst_uv = dst + uv_offset;
      // Y plane: 1080 rows × 1920 → into 1216-row × 2048-stride buffer
      for (int y = 0; y < src_height; y++) {
        memcpy(dst_y + y * stride, src_y + y * src_stride, src_width);
      }
      // UV plane: 540 rows × 1920 → into 608-row × 2048-stride buffer
      for (int y = 0; y < src_height / 2; y++) {
        memcpy(dst_uv + y * stride, src_uv + y * src_stride, src_width);
      }
      __sync_synchronize();
      g_in_frame_read = 0;
      // No torn-read check here: hal3_direct seq increments per frame.
      // The snapshot above is what we copied; if writer started a new frame
      // mid-memcpy that's the next iteration's concern.

      VisionIpcBufExtra ex = {.frame_id = cnt, .timestamp_sof = sof, .timestamp_eof = eof};
      rb->set_frame_id(cnt);
      vipc_server.send(rb, &ex);

      PUBLISH_FRAME(pm, "roadCameraState", initRoadCameraState, cnt, sof, eof);

      // Mirror road → wide (only if NO_WIDE is not set)
      if (publish_wide) {
        VisionBuf *wb = vipc_server.get_buffer(VISION_STREAM_WIDE_ROAD);
        if (wb && wb->addr) {
          memcpy(wb->addr, rb->addr, nv12_size);
          wb->set_frame_id(cnt);
          vipc_server.send(wb, &ex);
        }
        PUBLISH_FRAME(pm, "wideRoadCameraState", initWideRoadCameraState, cnt, sof, eof);
      }

      last_fid = cur_fid;  // use snapshot, not hdr (may have advanced)
      cnt++;

      uint64_t now = nanos_since_boot();
      if (now - last_log > 10000000000ULL) {
        LOG("frame=%u hal_fid=%u fps=%.1f drop=%u", cnt, hdr->frame_id, hdr->fps / 10.0f, hdr->dropped);
        last_log = now;
      }
    }

    // Cleanup this SHM session
    munmap(shm_ptr, shm_size);
    close(shm_fd);

    if (do_exit) break;
    LOG("Frame loop ended, will reconnect SHM...");
  }

  // Restore original SIGBUS handler
  sigaction(SIGBUS, &old_sa, NULL);
  LOG("Done");
}
