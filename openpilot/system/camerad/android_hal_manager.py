#!/usr/bin/env python3
"""
android_hal_manager.py - HAL3 Direct 进程管理 (Xiaomi Mi 8)

作为 openpilot PythonProcess 运行，由 SP manager 管理生命周期。
职责：
1. 确认/补位 Android binder 服务栈（systemd android-hal.service 应已启动）
2. 确认 HAL3 环境就绪（symlinks、OIS 固件、tuning path）
3. 启动并监控 hal3_direct 进程
4. hal3_direct 崩溃时自动重启
"""

import os
import subprocess
import time
import signal

CHROOT = "/data/android_root"
SHM_PATH = f"{CHROOT}/tmp/hal_camera"

HAL3_BIN = "/data/hal3_direct"
SSC_STUB = "/data/libssc_stub.so"

# Binder 服务定义：(名称, 进程匹配串, 启动命令)
SERVICES = [
    ("vndservicemanager", "vndservicemanager",
     f"chroot {CHROOT} /system/bin/sh -c '"
     "ANDROID_ROOT=/system ANDROID_DATA=/data "
     "LD_LIBRARY_PATH=/system/lib64:/vendor/lib64 "
     "/vendor/bin/vndservicemanager /dev/vndbinder &'"),

    ("servicemanager", "/system/bin/servicemanager",
     f"chroot {CHROOT} /system/bin/sh -c '"
     "ANDROID_ROOT=/system ANDROID_DATA=/data "
     "LD_LIBRARY_PATH=/system/lib64:/vendor/lib64 "
     "/system/bin/servicemanager &'"),

    ("hwservicemanager", "hwservicemanager",
     f"chroot {CHROOT} /system/bin/sh -c '"
     "ANDROID_ROOT=/system ANDROID_DATA=/data "
     "LD_LIBRARY_PATH=/system/lib64:/vendor/lib64 "
     "/system/bin/hwservicemanager &'"),

    ("allocator", "allocator@1.0-service",
     f"chroot {CHROOT} /system/bin/sh -c '"
     "ANDROID_ROOT=/system ANDROID_DATA=/data "
     "LD_LIBRARY_PATH=/system/lib64:/vendor/lib64 "
     "/vendor/bin/hw/vendor.qti.hardware.display.allocator@1.0-service &'"),
]

running = True
hal3_pid = None


def log(msg):
    print(f"[android_hal] {msg}", flush=True)


def run_cmd(cmd):
    try:
        subprocess.run(cmd, shell=True, timeout=10,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception as e:
        log(f"cmd failed: {e}")


def is_running(pattern):
    try:
        result = subprocess.run(
            ["pgrep", "-f", pattern],
            capture_output=True, timeout=5
        )
        return result.returncode == 0
    except Exception:
        return False


def ensure_base_services():
    """Start any missing binder services (fallback if systemd service failed)"""
    for name, pattern, cmd in SERVICES:
        if not is_running(pattern):
            log(f"starting {name} (fallback)...")
            run_cmd(f"sudo {cmd}")
            time.sleep(0.3)


def setup_hal3_env():
    """Ensure HAL3 Direct environment is ready (idempotent)"""
    # Symlinks — /vendor is tmpfs, recreate on every boot
    if not os.path.islink("/vendor/lib"):
        run_cmd("sudo ln -sf /data/android_root/vendor/lib /vendor/lib")
    if not os.path.islink("/vendor/etc/camera"):
        run_cmd("sudo ln -sf /data/android_root/vendor/etc/camera /vendor/etc/camera")
    if not os.path.islink("/dev/socket/property_service"):
        run_cmd("sudo ln -sf /dev/socket/leprop-service /dev/socket/property_service")

    # Binder permissions
    run_cmd("sudo chmod 666 /dev/hwbinder /dev/vndbinder")

    # Camera data directory
    os.makedirs("/data/vendor/camera", exist_ok=True)

    # OIS firmware
    ois_src = "/data/android_root/vendor/firmware"
    for fw in ["dipper_ois.prog", "dipper_ois.coeff"]:
        dest = f"/lib/firmware/{fw}"
        if not os.path.exists(dest):
            run_cmd(f"sudo cp {ois_src}/{fw} /lib/firmware/")

    # Tuning data path fix (doubled path workaround)
    link = "/data/android_root/vendor/lib/camera/vendor/lib/camera"
    if not os.path.islink(link):
        os.makedirs("/data/android_root/vendor/lib/camera/vendor/lib", exist_ok=True)
        run_cmd(f"ln -sf /data/android_root/vendor/lib/camera {link}")

    # SHM output directory
    os.makedirs(f"{CHROOT}/tmp", exist_ok=True)


def start_hal3_direct():
    """Start hal3_direct process, return True if SHM becomes ready"""
    global hal3_pid

    if not os.path.exists(HAL3_BIN):
        log(f"ERROR: {HAL3_BIN} missing!")
        return False
    if not os.path.exists(SSC_STUB):
        log(f"ERROR: {SSC_STUB} missing!")
        return False

    # Clean stale SHM
    run_cmd(f"sudo rm -f {SHM_PATH}")

    log("starting hal3_direct...")
    env = os.environ.copy()
    env["LD_PRELOAD"] = SSC_STUB
    env["LD_LIBRARY_PATH"] = "/vendor/lib:/vendor/lib/hw:/system/lib"

    logpath = "/data/hal3_direct.log"
    logf = open(logpath, "w")
    proc = subprocess.Popen(
        [HAL3_BIN],
        cwd="/data",
        env=env,
        stdout=logf,
        stderr=logf,
        start_new_session=True,
    )
    hal3_pid = proc.pid
    log(f"hal3_direct started (pid={hal3_pid})")

    # Wait for SHM to appear (up to 15s)
    for _ in range(30):
        if os.path.exists(SHM_PATH):
            log("hal3_direct SHM ready")
            return True
        # Check if process died
        try:
            os.kill(hal3_pid, 0)
        except OSError:
            log("hal3_direct died during startup!")
            hal3_pid = None
            return False
        time.sleep(0.5)

    log("WARNING: SHM not ready after 15s")
    return True  # process may still be initializing


def is_hal3_running():
    """Check if hal3_direct is alive"""
    global hal3_pid
    if hal3_pid is not None:
        try:
            os.kill(hal3_pid, 0)
            return True
        except OSError:
            hal3_pid = None
    return is_running("hal3_direct")


def ensure_all_running():
    """Monitor loop: restart any crashed service"""
    # Binder services
    for name, pattern, cmd in SERVICES:
        if not is_running(pattern):
            log(f"{name} died, restarting...")
            run_cmd(f"sudo {cmd}")
            time.sleep(0.3)

    # hal3_direct
    if not is_hal3_running():
        log("hal3_direct died, restarting...")
        start_hal3_direct()


def signal_handler(sig, frame):
    global running
    running = False


def cleanup():
    """Kill hal3_direct on exit"""
    global hal3_pid
    if hal3_pid is not None:
        log(f"stopping hal3_direct (pid={hal3_pid})")
        try:
            os.kill(hal3_pid, signal.SIGTERM)
            time.sleep(1)
            try:
                os.kill(hal3_pid, signal.SIGKILL)
            except OSError:
                pass
        except OSError:
            pass
        hal3_pid = None


def main():
    global running

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    log("=== Android HAL Manager starting (HAL3 Direct mode) ===")

    # Step 1: Wait for android-hal.service to finish environment setup
    log("waiting for android-hal.service...")
    for _ in range(30):
        if os.path.islink("/vendor/lib"):
            log("android-hal.service environment ready")
            break
        time.sleep(1)
    else:
        log("android-hal.service not ready, setting up environment ourselves")

    # Step 2: Ensure binder services
    ensure_base_services()

    # Step 3: Ensure HAL3 environment
    setup_hal3_env()

    # Step 4: 检查是否已有预热的 hal3_direct (由 slpi-start.sh 预启动)
    if is_hal3_running():
        log("hal3_direct already running (pre-warmed by slpi-start), waiting for SHM...")
        shm_ready = False
        for _ in range(30):  # 最多等 15 秒
            if os.path.exists(SHM_PATH):
                shm_ready = True
                break
            # 检查进程是否还活着
            if not is_hal3_running():
                log("pre-warmed hal3_direct died while waiting for SHM")
                break
            time.sleep(0.5)

        if shm_ready:
            log("hal3_direct SHM ready (pre-warmed), adopting...")
            try:
                result = subprocess.run(["pgrep", "-f", "hal3_direct"],
                                        capture_output=True, timeout=5)
                if result.returncode == 0:
                    hal3_pid = int(result.stdout.strip().split()[0])
                    log(f"adopted pre-warmed hal3_direct (pid={hal3_pid})")
            except Exception as e:
                log(f"adopt pid failed: {e}")
        else:
            # 预热进程崩溃或超时, 重新启动
            log("pre-warmed hal3_direct not ready, killing and restarting...")
            run_cmd("sudo pkill -9 -f hal3_direct")
            time.sleep(0.5)
            # Step 5: Start hal3_direct
            start_hal3_direct()
    else:
        # 没有预热进程, 正常 kill + restart
        run_cmd("sudo pkill -9 -f hal3_direct")
        time.sleep(0.5)

        # Step 5: Start hal3_direct
        start_hal3_direct()

    log("=== Initial setup complete, entering monitor loop ===")

    # Step 6: Monitor loop
    while running:
        time.sleep(5)
        try:
            ensure_all_running()
        except Exception as e:
            log(f"Monitor error: {e}")

    cleanup()
    log("=== Android HAL Manager stopped ===")


if __name__ == "__main__":
    main()
