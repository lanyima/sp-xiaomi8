#!/usr/bin/env python3
"""
fake_pandad_geely.py — 吉利缤越 CAN 数据模拟器
================================================
绕过真实 panda 硬件，直接向 cereal messaging 总线注入：
  - pandaStates  (10 Hz)  — 点火、安全模式、controlsAllowed
  - can          (50 Hz)  — PT 总线 (bus 0) + CAM 总线 (bus 2) 的所有必需消息

用途：在没有真车 / panda 的情况下启动 controls / card / selfdrived 进行调试。

────────────────────────────────────────────────────────────────────────────
快速开始（一行命令，自动完成所有准备）
────────────────────────────────────────────────────────────────────────────

  # SSH 上设备（必须带 -t 才有键盘交互）
  ssh -t comma@192.168.200.208

  # 一键启动（自动 setup + 跑 fake_pandad）
  cd /data/openpilot && PYTHONPATH=. python3 tools/sim/fake_pandad_geely.py --setup

  # 退出后清理还原
  python3 tools/sim/fake_pandad_geely.py --cleanup

────────────────────────────────────────────────────────────────────────────
完整原理（用 `--setup` 帮你自动完成的事）
────────────────────────────────────────────────────────────────────────────

1) 停掉 comma.service —— 避免真实 pandad 占用 cereal /can 和 /pandaStates topic
2) 设 Params:
     - CarPlatformBundle = {"platform":"GEELY_BINYUE","brand":"geely"}
       （让 card.py 跳过指纹识别，直接用吉利缤越平台）
3) 改 process_config.py —— 关掉真实 pandad（fake_pandad 顶上）
4) 在 launch_env.sh 末尾追加:
     - SKIP_FW_QUERY=1   （card 跳过 OBD-II VIN/FW 查询；否则会卡在 OBD 查询超时）
     - PASSIVE=1         （manager 接受无真 panda 的状态）
5) 重启 comma.service —— manager 启动 UI/camerad/controlsd/selfdrived/card 等
6) 在本进程里持续发布 can + pandaStates，UI 屏幕进入 onroad 模式显示路面画面

────────────────────────────────────────────────────────────────────────────
交互键（在前台终端运行时，按键即时生效，无需回车）
────────────────────────────────────────────────────────────────────────────
  q      — 退出（不做 cleanup, 需要单独跑 --cleanup）
  i      — 切换点火（ignition on/off）
  a      — 切换 ACC 激活（ACC_REQ 0→1→0）
  +/-    — 速度 +5 / -5 kph
  [/]    — 转向角 -5 / +5 度
  b      — 切换刹车
  s      — 打印当前状态

────────────────────────────────────────────────────────────────────────────
命令行参数
────────────────────────────────────────────────────────────────────────────
  --setup        准备环境（停 comma、设 Params、改 launch_env，再启动主循环）
  --cleanup      还原环境（撤销 setup 的所有改动，恢复正常 openpilot 运行）
  --no-restart   --setup 时不重启 comma（你想手动控时机用）
  无参数         直接进主循环（假定环境已 setup 好）

────────────────────────────────────────────────────────────────────────────
环境变量（可选，调初值）
────────────────────────────────────────────────────────────────────────────
  FAKE_SPEED_KPH   初始速度，默认 30
  FAKE_STEER_DEG   初始转向角，默认 0
  FAKE_ACC_SPEED   巡航设定速度 kph，默认 80

────────────────────────────────────────────────────────────────────────────
故障排查
────────────────────────────────────────────────────────────────────────────
- 屏幕一直 "sunnypilot Unavailable - Waiting to start":
    → card.py 卡在 fingerprint OBD 查询，确认 --setup 跑过且 SKIP_FW_QUERY=1 在 launch_env.sh
    → 或 `Params().get(\"CarPlatformBundle\")` 是否为 {"platform":"GEELY_BINYUE",...}

- 屏幕黑屏卡 comma logo:
    → magic.service inactive。`sudo systemctl start magic.service` + 重启 comma.service

- "Failed to initialize EGL":
    → /tmp/drmfd.sock 不存在。同上 magic.service 问题。

- termios.error: SSH 没用 -t。改用 `ssh -t comma@<ip>` 重连。

- vEgo 一直 0:
    → fake_pandad 没在跑，或者 process_config 没禁用真实 pandad（两者抢 pandaStates topic）
"""

import os
import sys
import time
import json
import threading
import select
import subprocess

# argparse 优先于 cereal 导入，方便 --setup/--cleanup 在无 sunnypilot env 时跑
ARGS = set(sys.argv[1:])
RUN_SETUP   = '--setup' in ARGS
RUN_CLEANUP = '--cleanup' in ARGS
NO_RESTART  = '--no-restart' in ARGS

# --csv=PATH : 回放真实抓包的 CAN（time,addr,bus,data）替代合成数据 —— 模拟原车启动
CSV_PATH = None
CSV_NO_LOOP = '--no-loop' in ARGS
for _a in sys.argv[1:]:
  if _a.startswith('--csv='):
    CSV_PATH = _a.split('=', 1)[1]

LAUNCH_ENV_PATH = '/data/openpilot/launch_env.sh'
PROCESS_CONFIG  = '/data/openpilot/openpilot/system/manager/process_config.py'
SETUP_MARKER    = '# xiaomi8 fake_pandad SETUP marker'
PANDAD_DISABLE_MARK  = '# fake_pandad takeover'
PANDAD_ENABLE_LINE   = 'PythonProcess("pandad", "openpilot.selfdrive.pandad.pandad", always_run)'
PANDAD_DISABLE_LINE  = 'PythonProcess("pandad", "openpilot.selfdrive.pandad.pandad", always_run, enabled=False),  # fake_pandad takeover'


def _run(cmd, **kw):
  return subprocess.run(cmd, shell=True, **kw)


def setup_env():
  """完成所有 fake_pandad 的前置准备工作。"""
  print("[setup] 1/5 停掉 comma.service ...")
  _run("sudo systemctl stop comma.service")
  time.sleep(2)

  print("[setup] 2/5 锁定 CarPlatformBundle = GEELY_BINYUE ...")
  # 这里必须用 venv 的 python（params 是 Cython 编译的 .so）
  # 直接在本进程里导入 Params 即可
  from openpilot.common.params import Params
  from openpilot.common.version import terms_version, training_version
  p = Params()
  p.put("CarPlatformBundle", {"platform": "GEELY_BINYUE", "brand": "geely"})

  # 跳过 onboarding（条款+训练），否则 hardwared 的 accepted_terms/completed_training
  # 条件不满足 → deviceState.started 永远 False → 即使点火也不 onroad。
  p.put("HasAcceptedTerms", terms_version)
  p.put("CompletedTrainingVersion", training_version)
  p.put_bool("DisableUpdates", True)

  print("[setup] 3/5 禁用真实 pandad（避免和 fake_pandad 抢 pandaStates topic）...")
  with open(PROCESS_CONFIG) as f:
    cfg = f.read()
  if PANDAD_DISABLE_MARK not in cfg and PANDAD_ENABLE_LINE + ',' in cfg:
    cfg = cfg.replace(PANDAD_ENABLE_LINE + ',', PANDAD_DISABLE_LINE)
    with open(PROCESS_CONFIG, 'w') as f:
      f.write(cfg)

  # v4.9.76: PASSIVE=0(主动模式) —— PASSIVE=1 会让 op 进"被动"少跑感知/UI,
  # 导致看不到行车画面(modelV2/车道线)。sp2025 能跑的最简版本本来就不设 PASSIVE。
  print("[setup] 4/5 加 SKIP_FW_QUERY=1 + PASSIVE=0 到 launch_env.sh ...")
  with open(LAUNCH_ENV_PATH) as f:
    env_sh = f.read()
  if SETUP_MARKER not in env_sh:
    env_sh += f"\n{SETUP_MARKER}\nexport SKIP_FW_QUERY=1\nexport PASSIVE=0\n"
    with open(LAUNCH_ENV_PATH, 'w') as f:
      f.write(env_sh)

  if NO_RESTART:
    print("[setup] 5/5 跳过 comma.service 重启 (--no-restart)")
    return
  print("[setup] 5/5 重启 comma.service ...")
  # Clear pyc to ensure process_config edits take effect
  _run("find /data/openpilot -name '*.pyc' -delete 2>/dev/null")
  _run("sudo systemctl restart comma.service")
  print("[setup] 等待 manager 起来 (20s) ...")
  time.sleep(20)
  print("[setup] ✓ 完成。pandaStates / can topic 现在由本进程接管。")
  print()


def cleanup_env():
  """还原 setup_env 的所有改动。"""
  print("[cleanup] 1/4 停 comma.service ...")
  _run("sudo systemctl stop comma.service")
  time.sleep(2)

  print("[cleanup] 2/4 还原 process_config.py（启用真实 pandad）...")
  with open(PROCESS_CONFIG) as f:
    cfg = f.read()
  if PANDAD_DISABLE_MARK in cfg:
    cfg = cfg.replace(PANDAD_DISABLE_LINE, PANDAD_ENABLE_LINE + ',')
    with open(PROCESS_CONFIG, 'w') as f:
      f.write(cfg)

  print("[cleanup] 3/4 移除 launch_env.sh 里的 SKIP_FW_QUERY/PASSIVE ...")
  with open(LAUNCH_ENV_PATH) as f:
    env_sh = f.read()
  if SETUP_MARKER in env_sh:
    lines = env_sh.split('\n')
    out = []
    skip = False
    for line in lines:
      if line.strip() == SETUP_MARKER:
        skip = True
        continue
      if skip and line.startswith('export '):
        continue
      skip = False
      out.append(line)
    with open(LAUNCH_ENV_PATH, 'w') as f:
      f.write('\n'.join(out))

  print("[cleanup] 4/4 清 CarPlatformBundle + 重启 comma.service ...")
  from openpilot.common.params import Params
  Params().remove("CarPlatformBundle")
  _run("find /data/openpilot -name '*.pyc' -delete 2>/dev/null")
  _run("sudo systemctl restart comma.service")
  print("[cleanup] ✓ 完成。")


# 模式分发：--cleanup 干完就退；--setup 干完后进入主循环
if RUN_CLEANUP:
  cleanup_env()
  sys.exit(0)
if RUN_SETUP:
  setup_env()


import termios
import tty

import cereal.messaging as messaging
from opendbc.can.packer import CANPacker
from openpilot.selfdrive.pandad.pandad_api_impl import can_list_to_can_capnp
from openpilot.common.realtime import Ratekeeper

# ──────────────────────────────────────────────
# 常量
# ──────────────────────────────────────────────
DBC_NAME    = "geely_binyue_pt"
PT_BUS      = 0
CAM_BUS     = 2
CTRL_HZ     = 100         # CAN 发送频率 (STEERING_MODULE 在 carstate 中要求 100Hz)
PANDA_HZ    = 10          # pandaStates 发送频率
PANDA_EVERY = CTRL_HZ // PANDA_HZ  # 每隔几帧发一次 pandaStates

# Geely 档位值（来自 DBC VAL_ 275 GEAR）
GEAR_D      = 6
GEAR_P      = 10
GEAR_R      = 11
GEAR_N      = 0


# ──────────────────────────────────────────────
# 可变状态（线程安全：GIL 保证原子读写）
# ──────────────────────────────────────────────
class FakeCarState:
    def __init__(self):
        self.speed_kph    = float(os.getenv("FAKE_SPEED_KPH",  "30"))
        self.steer_deg    = float(os.getenv("FAKE_STEER_DEG",   "0"))
        self.acc_speed    = float(os.getenv("FAKE_ACC_SPEED",  "80"))
        self.ignition     = True
        self.acc_req      = False      # True = 原车 ACC 激活
        self.brake        = False
        self.counter      = 0          # 通用计数器 0-15（循环）

    def tick(self):
        self.counter = (self.counter + 1) & 0xF


state = FakeCarState()


# ──────────────────────────────────────────────
# 键盘控制（非阻塞）
# ──────────────────────────────────────────────
def _getch_nonblock():
    """非阻塞读取单个字符，无输入时返回 None。"""
    dr, _, _ = select.select([sys.stdin], [], [], 0)
    if dr:
        return sys.stdin.read(1)
    return None


def keyboard_thread():
    """在独立线程中捕获键盘输入，修改 state。"""
    fd = sys.stdin.fileno()
    try:
        old = termios.tcgetattr(fd)
    except termios.error:
        print("[fake_pandad] 无 TTY（ssh 没用 -t？），键盘控制禁用。仍持续发布 CAN/pandaStates。")
        return
    try:
        tty.setcbreak(fd)
        while not stop_event.is_set():
            ch = _getch_nonblock()
            if ch is None:
                time.sleep(0.05)
                continue
            if ch == 'q':
                print("\n[fake_pandad] 退出")
                stop_event.set()
            elif ch == 'i':
                state.ignition = not state.ignition
                print(f"[fake_pandad] 点火: {'ON' if state.ignition else 'OFF'}")
            elif ch == 'a':
                state.acc_req = not state.acc_req
                print(f"[fake_pandad] ACC: {'激活' if state.acc_req else '待机'}")
            elif ch == '+' or ch == '=':
                state.speed_kph = min(state.speed_kph + 5, 200)
                print(f"[fake_pandad] 速度: {state.speed_kph:.0f} kph")
            elif ch == '-' or ch == '_':
                state.speed_kph = max(state.speed_kph - 5, 0)
                print(f"[fake_pandad] 速度: {state.speed_kph:.0f} kph")
            elif ch == '[':
                state.steer_deg -= 5
                print(f"[fake_pandad] 转向角: {state.steer_deg:.1f}°")
            elif ch == ']':
                state.steer_deg += 5
                print(f"[fake_pandad] 转向角: {state.steer_deg:.1f}°")
            elif ch == 'b':
                state.brake = not state.brake
                print(f"[fake_pandad] 刹车: {'按下' if state.brake else '释放'}")
            elif ch == 's':
                print(f"[fake_pandad] 状态: ign={state.ignition} acc_req={state.acc_req} "
                      f"speed={state.speed_kph:.0f}kph steer={state.steer_deg:.1f}° "
                      f"brake={state.brake}")
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


# ──────────────────────────────────────────────
# CAN 消息构建
# ──────────────────────────────────────────────
def build_pt_messages(packer, s):
    """构建 PT 总线 (bus 0) 上的所有必要 CAN 消息。

    重要: CANPacker.make_can_msg 接受 **物理值** (km/h, °等)，内部按 DBC scale
    转 raw bytes。**不要预先除以 scale**，否则等于应用两次（→ 数值爆表，vEgo
    被错算成几倍真实速度）。
    """
    msgs = []
    kph  = s.speed_kph
    # WHEEL_SPEED: 物理值就是 km/h，packer 内部按 DBC scale=0.007 转 raw
    msgs.append(packer.make_can_msg("WHEEL_SPEED", PT_BUS, {
        "WHEELSPEED_F": kph,
        "WHEELSPEED_B": kph,
    }))

    # STEERING_MODULE: 物理值是度数，packer 按 DBC scale=0.1 转 raw
    msgs.append(packer.make_can_msg("STEERING_MODULE", PT_BUS, {
        "STEER_ANGLE":  s.steer_deg,
        "STEER_RATE":   0,
        "SET_ME_2":     1,
        "SET_ME_1":     1,
        "CHECKSUM":     0,  # CANParser 不校验输入 checksum
    }))

    # STEERING_TORQUE
    msgs.append(packer.make_can_msg("STEERING_TORQUE", PT_BUS, {
        "MAIN_TORQUE":         0,
        "DRIVER_TORQUE_NO_ADAS": 0,
        "SET_ME_1":            0,
        "SET_ME_2":            0,
        "SET_ME_3":            0,
        "SET_ME_ACC":          0,
        "COUNTER":             s.counter,
        "CHECKSUM":            0,
    }))

    # BRAKE (BRAKE_PRESSURE)
    msgs.append(packer.make_can_msg("BRAKE", PT_BUS, {
        "BRAKE_PRESSURE": 500 if s.brake else 0,  # raw，scale=0.0002 → 0.1 bar
    }))

    # GAS_PEDAL
    msgs.append(packer.make_can_msg("GAS_PEDAL", PT_BUS, {
        "APPS_1":           0,        # 0 = 无踩油门
        "CRUISE_CONTROL_EN": 1,       # 巡航系统可用
        "CC_LIM_KMH":       0,
    }))

    # ENGINE
    msgs.append(packer.make_can_msg("ENGINE", PT_BUS, {
        "BRAKE_ENGAGED": 1 if s.brake else 0,
    }))

    # TRANSMISSION: D 档 = 6
    msgs.append(packer.make_can_msg("TRANSMISSION", PT_BUS, {
        "GEAR": GEAR_D,
    }))

    # PARKING_BRAKE
    msgs.append(packer.make_can_msg("PARKING_BRAKE", PT_BUS, {
        "BRAKE_PRESSED":           1 if s.brake else 0,
        "CAR_ON_HOLD":             0,
        "ESC_ON":                  1,   # ESC 正常工作
        "ENGAGING_TILL_RELEASE":   0,
    }))

    # SEATBELTS: LEFT_SIDE_SEATBELT_ACTIVE_LOW=1 表示已系上（active-low）
    msgs.append(packer.make_can_msg("SEATBELTS", PT_BUS, {
        "LEFT_SIDE_SEATBELT_ACTIVE_LOW":  1,
        "RIGHT_SIDE_SEATBELT_ACTIVE_LOW": 1,
    }))

    # DOOR_LEFT_SIDE / DOOR_RIGHT_SIDE: 全关
    msgs.append(packer.make_can_msg("DOOR_LEFT_SIDE", PT_BUS, {
        "FRONT_LEFT_DOOR":  0,
        "BACK_LEFT_DOOR":   0,
    }))
    msgs.append(packer.make_can_msg("DOOR_RIGHT_SIDE", PT_BUS, {
        "FRONT_RIGHT_DOOR": 0,
        "BACK_RIGHT_DOOR":  0,
    }))

    # LEFT_STALK: 无转向灯
    msgs.append(packer.make_can_msg("LEFT_STALK", PT_BUS, {
        "LEFT_SIGNAL":  0,
        "RIGHT_SIGNAL": 0,
        "GENERIC_TOGGLE": 0,
    }))

    # POWER_MODE: MODE=3 表示点火完全启动 (RUN)
    # carstate 把这条消息列在 pt_messages 里做新鲜度检查，缺它会导致 canValid=False
    msgs.append(packer.make_can_msg("POWER_MODE", PT_BUS, {
        "MODE": 3,
    }))

    # BSM_ADAS: CP.enableBsm 默认 True，所以 carstate 把它加入 pt_messages
    # 必须发，否则 canValid=False → 弹 "Unknown Vehicle Variant" (canError 警报)
    msgs.append(packer.make_can_msg("BSM_ADAS", PT_BUS, {
        "RIGHT_APPROACH":         0,
        "RIGHT_APPROACH_WARNING": 0,
        "LEFT_APPROACH":          0,
        "LEFT_APPROACH_WARNING":  0,
        "COUNTER":                s.counter & 0xF,
    }))

    # FCW
    msgs.append(packer.make_can_msg("FCW", PT_BUS, {
        "STOCK_FCW_TRIGGERED": 0,
    }))

    # ACC_BUTTONS: 无按键
    msgs.append(packer.make_can_msg("ACC_BUTTONS", PT_BUS, {
        "SET_BUTTON":            0,
        "RES_BUTTON":            0,
        "CRUISE_BTN":            0,
        "SET_ME_BUTTON_PRESSED": 0,
        "COUNTER":               s.counter,
        "CHECKSUM":              0,
    }))

    return msgs


def build_cam_messages(packer, s):
    """构建 CAM 总线 (bus 2) 上的所有必要 CAN 消息。"""
    msgs = []

    # ACC_CMD: CRUISE_ENABLE=1 才会让 openpilot 显示 ACC 可用
    # ACC_REQ=1 && NOT_ACC_REQ=0 → stock_acc_active=True (原车 ACC 已激活)
    msgs.append(packer.make_can_msg("ACC_CMD", CAM_BUS, {
        "CRUISE_ENABLE":   1,
        "ACC_REQ":         1 if s.acc_req else 0,
        "NOT_ACC_REQ":     0 if s.acc_req else 1,
        "BRAKE_ENGAGED":   1 if s.brake else 0,
        "SET_ME_1":        1,
        "RISING_ENGAGE":   0,
        "UNKNOWN1":        0,
        "STANDSTILL2":     1 if s.speed_kph < 1 else 0,
        "SET_ME_X6A":      0x6A,
        "CMD_OFFSET1":     142,   # raw offset (DBC offset -142 → raw=142 → value=0)
        "CMD_OFFSET2":     140,
        "CMD":             138,
        "STATIONARY":      1 if s.speed_kph < 1 else 0,
        "NOT_ACC_REQ":     0 if s.acc_req else 1,
        "MOTION_CONTROL":  0,
        "COUNTER":         s.counter,
        "CHECKSUM":        0,
    }))

    # PCM_BUTTONS: 巡航设定速度（直接 km/h）
    msgs.append(packer.make_can_msg("PCM_BUTTONS", CAM_BUS, {
        "ACC_SET_SPEED":          int(s.acc_speed),
        "ACC_ON_OFF_BUTTON":      0,
        "ACC_SET":                0,
        "GAS_OVERRIDE":           0,
        "DISTANCE_SETTING_BUTTON": 0,
        "SET_DISTANCE":           1,   # 1 bar 跟车距离
        "ICC_ON":                 0,
        "SET_ME_1":               0,
        "DRIVING_MODES":          0,
        "COUNTER":                s.counter,
        "CHECKSUM":               0,
    }))

    # ADAS_LKAS: 模拟原车摄像头发出的 LKAS 消息（无转向请求）
    msgs.append(packer.make_can_msg("ADAS_LKAS", CAM_BUS, {
        "STEER_CMD":              0,
        "STEER_DIR":              0,
        "LKAS_ENGAGED1":          0,
        "LKA_ENABLE":             1,   # LKAS 系统可用
        "LKS_STATUS":             0,
        "LKAS_LINE_ACTIVE":       0,
        "LDW_READY":              1,
        "SET_ME_1":               1,
        "LDW_STEERING":           0,
        "HAND_ON_WHEEL_WARNING":  0,
        "WHEEL_WARNING_CHIME":    0,
        "STOCK_LKS_AUX":          0,
        "LKS_WARNING_AUDIO_TYPE": 0,
        "LKS_WARNING_TACTILE_TYPE": 0,
        "LKS_ASSIST_MODE":        0,
        "COUNTER":                s.counter,
        "CHECKSUM":               0,
    }))

    # LKAS: 车道线状态
    msgs.append(packer.make_can_msg("LKAS", CAM_BUS, {
        "LEFT_LANE_VISIBLE_DISENGAGE":  0,   # 0 = 车道线可见
        "RIGHT_LANE_VISIBLE_DISENGAGE": 0,
        "LANE_DEPARTURE_AUDIO_LEFT":    0,
        "LANE_DEPARTURE_AUDIO_RIGHT":   0,
        "STEER_REQ_LEFT":               0,
        "STEER_REQ_RIGHT":              0,
        "STEER_REQ_MAJOR":              0,
        "LLANE_CHAR":                   0,
        "RLANE_CHAR":                   0,
        "CURVATURE":                    0,
    }))

    # ADAS_LEAD_DETECT: 无前车
    msgs.append(packer.make_can_msg("ADAS_LEAD_DETECT", CAM_BUS, {
        "LEAD_DISTANCE":  0,
        "IS_LEAD1":       0,
        "IS_LEAD2":       0,
        "LEAD_TOO_NEAR":  0,
        "NEW_SIGNAL_1":   0,
        "NEW_SIGNAL_2":   0,
    }))

    # ADAS_CAR_DETECT: 无侧方车辆
    msgs.append(packer.make_can_msg("ADAS_CAR_DETECT", CAM_BUS, {
        "LEFT_LANE_CAR_EXIST":  0,
        "LEFT_LANE_CAR_DIST":   0,
        "RIGHT_LANE_CAR_EXIST": 0,
        "RIGHT_LANE_CAR_DIST":  0,
        "NEW_SIGNAL_1":         126,  # raw offset (DBC offset -126 → raw=126 → value=0)
        "SET_ME_X7E":           126,
    }))

    return msgs


# ──────────────────────────────────────────────
# pandaStates 构建
# ──────────────────────────────────────────────
def send_panda_states(pm, s):
    """发布 pandaStates 消息，模拟 black panda 已连接、点火、控制允许。
    防御式赋值: 不同 fork/cereal 版本字段有差异(如 sp2025 无 controlsAllowedLateral),
    字段不存在就跳过, 避免 AttributeError 崩溃(否则 sim 一起就崩, 永不 onroad)。"""
    dat = messaging.new_message('pandaStates', 1)
    dat.valid = True
    ps = dat.pandaStates[0]

    def _set(field, value):
        try:
            setattr(ps, field, value)
        except Exception:
            pass

    _set("ignitionLine", s.ignition)
    _set("ignitionCan", False)
    _set("pandaType", "blackPanda")
    _set("safetyModel", "geely")
    _set("safetyParam", 0)
    _set("alternativeExperience", 0)
    _set("controlsAllowed", True)
    _set("controlsAllowedLateral", True)          # 新版 sunnypilot 才有
    _set("controlsAllowedLongitudinal", True)     # 新版 sunnypilot 才有
    _set("faultStatus", 0)
    _set("powerSaveEnabled", False)
    _set("heartbeatLost", False)
    pm.send('pandaStates', dat)


# ──────────────────────────────────────────────
# CSV 回放（真实原车抓包）—— 模拟原车启动
# ──────────────────────────────────────────────
PANDA_DT = 0.1  # pandaStates 周期 (10 Hz)


def _parse_csv_line(line):
    """解析一行 'time,addr,bus,data' → (t, addr, bus, data_bytes)，坏行返回 None。"""
    parts = line.rstrip('\n').split(',')
    if len(parts) != 4:
        return None
    try:
        t    = float(parts[0])
        addr = int(parts[1], 16)
        # bus & 0x7F：抹掉 panda TX 回显标志 (128+N)，让 cam(130)→2、pt(128)→0 落到 openpilot 期望的总线
        bus  = int(parts[2]) & 0x7F
        d    = parts[3].strip()
        data = bytes.fromhex(d[2:] if d[:2].lower() == '0x' else d)
    except Exception:
        return None
    return t, addr, bus, data


def replay_csv(pm, path, s, loop=True):
    """流式回放 CSV 的 CAN，按录制时间戳节奏发到 can topic；同时 10Hz 发 pandaStates（点火）。

    关键: 真 panda(boardd) 是把一段时间窗内收到的所有 CAN 帧**攒成一条 can 消息、~100Hz 发**。
    如果按 CSV 每个毫秒时间戳各发一条(可到 ~1000Hz), card_thread 会被这个 can 洪泛拉到几百 Hz
    空转 convert_carControlSP(每步一次 capnp to_dict, 很贵) → card 吃满一个核, 把 UI 饿死/卡顿。
    所以这里**限制 can 发送为 100Hz**(SEND_DT=10ms), 期间攒帧, 到点一次性发, 模拟真 panda。
    """
    SEND_DT = 0.01  # 100Hz — 跟真 panda 一致, 别把 card 拉爆
    print(f"[fake_pandad] CSV 回放: {path}  loop={loop} (can 批量 {int(1/SEND_DT)}Hz)")
    while not stop_event.is_set():
        with open(path) as f:
            f.readline()  # 跳过表头
            start = time.monotonic()
            last_send = start
            last_panda = start
            t0 = None
            batch = []

            def _tick():
                # 到点就把攒的 CAN 批量发出去(100Hz) + 维持 pandaStates(10Hz)
                nonlocal batch, last_send, last_panda
                now_m = time.monotonic()
                if batch and now_m - last_send >= SEND_DT:
                    pm.send('can', can_list_to_can_capnp(batch))
                    batch = []
                    last_send = now_m
                if now_m - last_panda >= PANDA_DT:
                    send_panda_states(pm, s)
                    last_panda = now_m

            for line in f:
                if stop_event.is_set():
                    return
                row = _parse_csv_line(line)
                if row is None:
                    continue
                t, addr, bus, data = row
                if t0 is None:
                    t0 = t
                # 按录制时刻给这一帧节奏; 等待期间照常 100Hz 发批 + 10Hz pandaStates
                while not stop_event.is_set():
                    now = time.monotonic() - start
                    if now >= (t - t0):
                        break
                    _tick()
                    time.sleep(min((t - t0) - now, 0.003))
                batch.append((addr, data, bus))
                _tick()
            # 收尾最后一批
            if batch and not stop_event.is_set():
                pm.send('can', can_list_to_can_capnp(batch))
        print(f"[fake_pandad] CSV 回放完一遍{'，循环重播' if loop else ''}")
        if not loop:
            stop_event.set()
            return


# ──────────────────────────────────────────────
# 主循环
# ──────────────────────────────────────────────
stop_event = threading.Event()


def main():
    print("[fake_pandad_geely] 启动 —— 按 's' 查看状态，'q' 退出")
    print("  i=点火切换  a=ACC激活  +/-=速度±5kph  [/]=转向角±5°  b=刹车")

    pm = messaging.PubMaster(['can', 'pandaStates'])

    # 键盘线程（CSV 模式下仍可用 'i' 切点火、'q' 退出）
    kb_thread = threading.Thread(target=keyboard_thread, daemon=True)
    kb_thread.start()

    # ── CSV 回放模式：喂真实原车抓包 ──
    if CSV_PATH is not None:
        replay_csv(pm, CSV_PATH, state, loop=not CSV_NO_LOOP)
        return

    # ── 合成模式：程序生成 CAN ──
    packer = CANPacker(DBC_NAME)
    rk    = Ratekeeper(CTRL_HZ, print_delay_threshold=None)
    frame = 0

    while not stop_event.is_set():
        s = state
        s.tick()

        # ── pandaStates (10 Hz) ──
        if frame % PANDA_EVERY == 0:
            send_panda_states(pm, s)

        # ── CAN (50 Hz) ──
        msgs  = build_pt_messages(packer, s)
        msgs += build_cam_messages(packer, s)
        pm.send('can', can_list_to_can_capnp(msgs))

        frame += 1
        rk.keep_time()


if __name__ == "__main__":
    main()
