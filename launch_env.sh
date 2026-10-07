#!/usr/bin/env bash

export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
export NUMEXPR_NUM_THREADS=1
export OPENBLAS_NUM_THREADS=1
export VECLIB_MAXIMUM_THREADS=1

# models get lower priority than ui
# - ui is ~5ms
# - modeld is 20ms
# - DM is 10ms
# in order to run ui at 60fps (16.67ms), we need to allow
# it to preempt the model workloads. we have enough
# headroom for this until ui is moved to the CPU.
export QCOM_PRIORITY=12

if [ -z "$AGNOS_VERSION" ]; then
  export AGNOS_VERSION="19.5"
fi

export STAGING_ROOT="/data/safe_staging"

# === xiaomi8 environment ===
export NO_DM=1                   # no driver-facing camera
export NO_WIDE=1                 # no wide road camera

# xiaomi8 V4L2 camera (2026-08-31): run IMX363 in 2x2 binning 1920x1080 so the
# IFE scaler is 1:1 (fixes the alternating-column stripe artifact). camerad_v4l2
# reads this via getenv("IMX363_BINNED").
export IMX363_BINNED=1
# V4L2 AE uses IMX363's real analogue gain stops. Do not add a fixed digital
# lift afterwards: a same-scene device A/B showed 2x destroys fine texture for
# minimal brightness benefit, particularly when exposure is already at 20 Hz.
export IMX363_DIGITAL_GAIN=1 # factory-like: no fixed digital lift
export IMX363_TARGET_GREY_FACTOR=0.002 # daylight-safe AE target; low-light is exposure-limited at 20fps
export IMX363_GAMMA_PROFILE=1 # low-light default; auto mode selects daylight curve only at short exposure
# Adaptive ABF is controlled per frame by analogue gain: at 1x it preserves
# daylight detail, while >=2x restores the Semco factory ABF34 noise profile.
# Set IMX363_ABF_MODE=on/off only for a controlled diagnostic A/B.
export IMX363_ABF_MODE=auto
export IMX363_20FPS=1  # match roadCameraState/model cadence
# Night driving profile: cap integration at ~25 ms so vehicle motion does not
# smear road markings and distant detail.  IMX363 then uses its real 16x analog
# stop if needed; fixed digital gain remains disabled above.
export IMX363_MOTION_SAFE_NIGHT=1
export IMX363_AE_RESPONSE_SECONDS=0.35 # motion-boundary AE smoothing
export IMX363_AE_GREY_DEADBAND=0.045 # reject metering quantization
# Keep the validated default Gamma until the AEC/DRC trigger tree is decoded.
# IMX363_GAMMA_PROFILE=2 remains a manual diagnostic A/B only; a fixed
# conservative curve made ordinary indoor scenes materially underexposed.
export NO_FAN=1                  # no fan hardware
export NO_LOCATIOND_TEMP=1       # skip locationd temperature compensation
# AGNOS on Xiaomi 8 exposes an incomplete CPU mask to Python. Keep the
# inherited 0-7 scheduler mask if a requested Gold core is not visible.
export XIAOMI8_KEEP_INHERITED_AFFINITY=1

# Memory optimisation: glibc tuning (jemalloc not in AGNOS 17.2)
export MALLOC_ARENA_MAX=2
export MALLOC_TRIM_THRESHOLD_=65536
export MALLOC_MMAP_THRESHOLD_=65536

# Xiaomi 8: disable obsolete CSID per-frame debug IRQ/HBI-VBI diagnostics.
if command -v sudo >/dev/null 2>&1 \&\& sudo -n test -e /sys/kernel/debug/camera_ife/ife_csid_debug 2>/dev/null; then
  sudo -n sh -c 'printf 0 > /sys/kernel/debug/camera_ife/ife_csid_debug' >/dev/null 2>&1 || true
fi





















