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
export NO_FAN=1                  # no fan hardware
export NO_LOCATIOND_TEMP=1       # skip locationd temperature compensation

# Memory optimisation: glibc tuning (jemalloc not in AGNOS 17.2)
export MALLOC_ARENA_MAX=2
export MALLOC_TRIM_THRESHOLD_=65536
export MALLOC_MMAP_THRESHOLD_=65536
