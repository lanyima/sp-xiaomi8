#!/usr/bin/env python3
"""
replay_can_csv.py — 把真实录制的 CAN 日志（csv）回放成 cereal `can` topic，
让 openpilot 离线就能跑车型识别 / carstate / selfdrived / UI。

跟 fake_pandad_geely.py 的目标一致，但用真实数据：
  - 全部消息频率/checksum/counter 都是真车的
  - 自动 loop 播放，让进程持续保持 canValid=True
  - 同时发布 pandaStates（ignitionLine=True, safetyModel=<brand>），manager
    判定 onroad → 启动 card/selfdrived/controlsd

CSV 格式（来自 cabana / panda 录制）：
  time,addr,bus,data
  0.000,0x120,0,0xFA00000FA0FA05DE
  0.000,0x121,0,0x7D01028C000005C8
  ...
  - time: 相对秒（float）
  - addr: 0x... 十六进制 CAN ID
  - bus:  0 / 2 / 130(=2|0x80 表示 panda 转发) — bus>=128 跳过（那是发送方向）
  - data: 0x... 8 字节十六进制 payload

用法：
  # 第一次：禁用真实 pandad（让出 can/pandaStates topic）
  sudo systemctl stop comma
  python3 -m tools.sim.replay_can_csv --setup

  # 启动回放（持续 loop，Ctrl-C 退出）
  python3 -m tools.sim.replay_can_csv path/to/LEIDA5.csv

  # 选项
  --speed 1.0     1.0 = 实时；0.5 = 半速；2.0 = 双倍
  --safety geely  panda 安全模型名（默认 geely）
  --no-loop       播完不循环
  --cleanup       恢复真实 pandad 后退出
"""
import argparse
import csv
import os
import re
import sys
import time
from collections import defaultdict

import cereal.messaging as messaging
from openpilot.common.realtime import Ratekeeper

# 2026-08-25: 嵌套树探测(真代码在 /data/openpilot/openpilot/ 时, 外层 /data/openpilot/ 只有
# launch_env.sh)。踩过的坑: 写死 /data/openpilot 在嵌套树上 process_config.py 找不到、
# manager.py 也找不到, setup() 静默失败或崩。
REPO_ROOT = '/data/openpilot'
_nested_pc = os.path.join(REPO_ROOT, 'openpilot', 'system', 'manager', 'process_config.py')
CODE_ROOT = os.path.join(REPO_ROOT, 'openpilot') if os.path.isfile(_nested_pc) else REPO_ROOT
LAUNCH_ENV_PATH = os.path.join(REPO_ROOT, 'launch_env.sh')
PROCESS_CONFIG = os.path.join(CODE_ROOT, 'system', 'manager', 'process_config.py')
PANDAD_DISABLE_MARK = '# replay_can_csv takeover'
SIM_ENV_MARK = '# replay_can_csv takeover'
BUNDLE_BACKUP = '/data/replay_can_csv.CarPlatformBundle.backup'


TICK_HZ = 100             # 回放时间分辨率（10ms 桶）
PANDA_HZ = 10
PANDA_EVERY = TICK_HZ // PANDA_HZ


def parse_hex_data(s: str) -> bytes:
    """0xFA00000FA0FA05DE → b'\\xfa\\x00...'"""
    s = s.strip().lower()
    if s.startswith("0x"):
        s = s[2:]
    if len(s) % 2:
        s = "0" + s
    return bytes.fromhex(s)


def _cache_path(csv_path: str) -> str:
    """同目录下的 .replay_cache.pkl"""
    base = os.path.basename(csv_path) + ".replay_cache.pkl"
    return os.path.join(os.path.dirname(os.path.abspath(csv_path)) or ".", base)


def load_csv(path: str):
    """返回 [(addr, data, bus), ...] 按 10ms 桶聚合的列表，长度 = total_buckets。

    优先读 .replay_cache.pkl（同目录），不存在时解析 csv 后落盘缓存。
    """
    import pickle
    cache = _cache_path(path)
    if os.path.exists(cache) and os.path.getmtime(cache) > os.path.getmtime(path):
        print(f"[replay] 命中缓存 {cache} ...")
        with open(cache, "rb") as f:
            buckets = pickle.load(f)
        total_t = len(buckets) / TICK_HZ
        print(f"[replay] 缓存 {len(buckets)} 桶, 时长 ~{total_t:.1f}s")
        return buckets

    print(f"[replay] 解析 CSV {path} ...")
    tmp_buckets: dict[int, list] = defaultdict(list)
    n_in = n_skip = 0
    with open(path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            n_in += 1
            try:
                t    = float(row["time"])
                addr = int(row["addr"], 16) if row["addr"].startswith("0x") else int(row["addr"])
                bus  = int(row["bus"])
                if bus >= 128:
                    n_skip += 1
                    continue
                data = parse_hex_data(row["data"])
                if not (1 <= len(data) <= 64):
                    n_skip += 1
                    continue
                tmp_buckets[int(t * TICK_HZ)].append((addr, data, bus))
            except Exception:
                n_skip += 1

    if not tmp_buckets:
        return []
    total = max(tmp_buckets) + 1
    buckets = [tmp_buckets.get(i, ()) for i in range(total)]   # 转 list，常数索引
    total_t = total / TICK_HZ
    print(f"[replay] 共 {n_in} 行, 跳过 {n_skip}, 时长 ~{total_t:.1f}s, 桶数 {total}")

    try:
        with open(cache, "wb") as f:
            pickle.dump(buckets, f, protocol=pickle.HIGHEST_PROTOCOL)
        print(f"[replay] 已缓存到 {cache}")
    except Exception as e:
        print(f"[replay] 写缓存失败: {e}")
    return buckets


def prebuild_can_msgs(buckets):
    """把每个桶预先打包成 cereal 'can' 消息字节，回放时只 send 不构造。

    返回 [bytes_or_None]，None 表示该桶无 can 消息（仍要 tick 不 send）。
    """
    print(f"[replay] 预构建 {len(buckets)} 条 can 消息 ...")
    out = []
    for group in buckets:
        if not group:
            out.append(None)
            continue
        m = messaging.new_message("can", len(group))
        for i, (addr, data, bus) in enumerate(group):
            cd = m.can[i]
            cd.address = addr
            cd.dat = data
            cd.src = bus
        out.append(m.to_bytes())  # bytes 占用比 capnp builder 小
    print(f"[replay] 预构建完成（{sum(1 for x in out if x is not None)} 个有效桶）")
    return out


def build_panda_msg(safety_model: str, ignition: bool):
    """伪造 pandaStates，让 manager 判定 onroad（pandaType != unknown 是必要条件）"""
    msg = messaging.new_message("pandaStates", 1)
    msg.valid = True              # 否则 selfdrived 弹 usbError 警报
    ps = msg.pandaStates[0]
    ps.pandaType    = "blackPanda"
    ps.ignitionLine = ignition
    ps.ignitionCan  = False
    ps.controlsAllowed = True
    # safetyModel 是 enum，直接传 string；capnp 会按名称匹配
    try:
        ps.safetyModel = safety_model
    except Exception:
        ps.safetyModel = "geely"
    ps.safetyParam = 0
    ps.alternativeExperience = 0
    return msg


def _toggle_pandad(disable: bool) -> bool:
    """process_config.py 里 pandad 那行加/去 enabled=False。正则匹配, 不管模块路径是
    "selfdrive.pandad.pandad" 还是 "openpilot.selfdrive.pandad.pandad"(分支不同不一样)。
    ★2026-08-25 加的: 之前 setup() 只临时 pkill 一下 pandad, 没改 process_config.py,
    manager 重启时(always_run)又把真 pandad 拉起来了 —— 真 pandad 和本工具的合成
    pandaStates 同一 topic 打架, 导致 selfdrived 报 "Controls Mismatch"(safety_mismatch/
    controlsAllowed 时有时无)。必须真的在 process_config.py 里禁掉。"""
    if not os.path.isfile(PROCESS_CONFIG):
        print(f"[warn] process_config.py 不存在: {PROCESS_CONFIG}")
        return False
    with open(PROCESS_CONFIG) as f:
        cfg = f.read()
    if disable:
        if PANDAD_DISABLE_MARK in cfg:
            return True
        # Mi 8 gates the real USB panda with enabled=REAL_PANDA_ENABLED.  The
        # old pattern only matched a bare always_run line, so CSV replay left
        # real pandad alive and two publishers fought over pandaStates/can.
        pat = re.compile(r'PythonProcess\("pandad",\s*"([\w.]+)",\s*always_run,\s*enabled=REAL_PANDA_ENABLED\)(,?)')
        new_cfg, n = pat.subn(
            lambda m: f'PythonProcess("pandad", "{m.group(1)}", always_run, enabled=False){m.group(2)}  {PANDAD_DISABLE_MARK}',
            cfg, count=1)
        if not n:
            # Compatibility with packages that use the upstream bare form.
            pat = re.compile(r'PythonProcess\("pandad",\s*"([\w.]+)",\s*always_run\)(,?)')
            new_cfg, n = pat.subn(
                lambda m: f'PythonProcess("pandad", "{m.group(1)}", always_run, enabled=False){m.group(2)}  {PANDAD_DISABLE_MARK}',
                cfg, count=1)
    else:
        if PANDAD_DISABLE_MARK not in cfg:
            return True
        pat = re.compile(
            r'PythonProcess\("pandad",\s*"([\w.]+)",\s*always_run,\s*enabled=False\)(,?)\s*' +
            re.escape(PANDAD_DISABLE_MARK))
        # Restore the Mi 8 USB-panda gate.  It is intentionally not changed to
        # unconditional always_run: that reintroduced the phone boot loop when
        # a panda was connected at power-on.
        new_cfg, n = pat.subn(
            lambda m: f'PythonProcess("pandad", "{m.group(1)}", always_run, enabled=REAL_PANDA_ENABLED){m.group(2)}',
            cfg, count=1)
    if n:
        with open(PROCESS_CONFIG, 'w') as f:
            f.write(new_cfg)
        return True
    print("[warn] 没找到 pandad 那一行, process_config.py 格式变了?")
    return False


def setup(safety_model: str):
    """停掉真实 pandad、让出 can+pandaStates topic、确保 card 能真正 fingerprint。"""
    print("[setup] 1/4 stop comma.service ...")
    os.system("sudo systemctl stop comma 2>/dev/null")
    time.sleep(2)
    print("[setup] 2/4 process_config.py 里禁掉真 pandad(避免和本工具的合成 pandaStates 打架)...")
    _toggle_pandad(disable=True)
    print("[setup] 3/4 设置 CSV 回放环境（跳过 FW 查询 + SIMULATION）...")
    with open(LAUNCH_ENV_PATH) as f:
        env_sh = f.read()
    if SIM_ENV_MARK not in env_sh:
        with open(LAUNCH_ENV_PATH, 'a') as f:
            f.write('\n# replay_can_csv takeover\nexport SKIP_FW_QUERY=1  # replay_can_csv takeover\n'
                    'export PASSIVE=0  # replay_can_csv takeover\n'
                    'export SIMULATION=1  # replay_can_csv takeover\n')
    # A known platform avoids an OBD fingerprint/FW-query timeout during
    # replay. Preserve a pre-existing bundle byte-for-byte for cleanup.
    if safety_model == 'geely' and not os.path.exists(BUNDLE_BACKUP):
        try:
            from openpilot.common.params import Params
            params = Params()
            old = params.get('CarPlatformBundle')
            if old is not None:
                with open(BUNDLE_BACKUP, 'wb') as f:
                    f.write(old)
            params.put('CarPlatformBundle', {'platform': 'GEELY_BINYUE', 'brand': 'geely'})
        except Exception as e:
            print(f"[warn] 无法设置 CarPlatformBundle: {e}")
    print("[setup] 4/4 清 pyc + 重启 comma.service(用它而不是裸 manager.py —— 才会正确 source"
          " launch_env.sh 的环境变量, 且嵌套树上路径才对)...")
    os.system(f"find {REPO_ROOT} -name '*.pyc' -delete 2>/dev/null")
    os.system("sudo systemctl restart comma 2>/dev/null")
    print("[setup] 等 manager/card 起来 (20s) ...")
    time.sleep(20)
    print("[setup] ✓ 完成。现在另起一终端跑 replay_can_csv.py <csv>")


def cleanup():
    """恢复真实 pandad + SKIP_FW_QUERY + 清 CarPlatformBundle。"""
    print("[cleanup] 1/4 stop comma.service ...")
    os.system("sudo systemctl stop comma 2>/dev/null")
    time.sleep(2)
    print("[cleanup] 2/4 process_config.py 里恢复真 pandad ...")
    _toggle_pandad(disable=False)
    print("[cleanup] 3/4 移除 CSV 模拟环境 ...")
    with open(LAUNCH_ENV_PATH) as f:
        lines = f.read().split('\n')
    lines = [l for l in lines if 'replay_can_csv takeover' not in l]
    with open(LAUNCH_ENV_PATH, 'w') as f:
        f.write('\n'.join(lines))
    print("[cleanup] 4/4 恢复 CarPlatformBundle + 重启 comma.service ...")
    try:
        from openpilot.common.params import Params
        params = Params()
        if os.path.exists(BUNDLE_BACKUP):
            with open(BUNDLE_BACKUP, 'rb') as f:
                original_bundle = f.read()
            # Params.get() may legitimately have returned b'' before replay.
            # Do not persist an empty bundle: that is neither a valid capnp
            # bundle nor the absence-of-parameter state we are restoring.
            if original_bundle:
                params.put('CarPlatformBundle', original_bundle)
            else:
                params.remove('CarPlatformBundle')
            os.unlink(BUNDLE_BACKUP)
        else:
            params.remove('CarPlatformBundle')
    except Exception as e:
        print(f"[warn] 无法恢复 CarPlatformBundle: {e}")
    os.system(f"find {REPO_ROOT} -name '*.pyc' -delete 2>/dev/null")
    os.system("sudo systemctl restart comma 2>/dev/null")
    print("[cleanup] ✓")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="?", help="CAN 录制 csv 文件路径")
    ap.add_argument("--setup",   action="store_true", help="停掉真实 pandad 并准备")
    ap.add_argument("--cleanup", action="store_true", help="恢复真实 pandad")
    ap.add_argument("--speed",  type=float, default=1.0, help="回放速度（1.0=实时, 默认 1.0）")
    ap.add_argument("--safety", default="geely",      help="panda 安全模型名（默认 geely）")
    ap.add_argument("--no-loop", action="store_true",  help="放完一遍就退出（默认 loop）")
    args = ap.parse_args()

    if args.setup:   setup(args.safety);   return
    if args.cleanup: cleanup(); return
    if not args.csv:
        ap.print_help(); sys.exit(2)

    buckets = load_csv(args.csv)
    if not buckets:
        print("[replay] CSV 为空或解析失败"); sys.exit(1)

    pm = messaging.PubMaster(["can", "pandaStates"])

    # 预构建：消除热路径上每帧 capnp 构造开销
    can_msgs = prebuild_can_msgs(buckets)
    panda_msg_bytes = build_panda_msg(args.safety, ignition=True).to_bytes()
    del buckets   # 释放原始解析结果，留下 bytes 列表

    rk = Ratekeeper(int(TICK_HZ * args.speed), print_delay_threshold=None)

    total = len(can_msgs)
    print(f"[replay] 开始回放 (speed={args.speed}x, loop={'off' if args.no_loop else 'on'}, ticks={total})")

    # 直接用底层 socket，绕开每次 .send 的 dict 查找
    can_sock   = pm.sock["can"]
    panda_sock = pm.sock["pandaStates"]

    panda_counter = 0
    while True:
        for tick in range(total):
            m = can_msgs[tick]
            if m is not None:
                can_sock.send(m)
            if panda_counter % PANDA_EVERY == 0:
                panda_sock.send(panda_msg_bytes)
            panda_counter += 1
            rk.keep_time()
        if args.no_loop:
            break
        print("[replay] 一轮结束，loop 重播...")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[replay] 退出")
