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
import sys
import time
from collections import defaultdict

import cereal.messaging as messaging
from openpilot.common.realtime import Ratekeeper


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


def setup():
    """停掉真实 pandad / openpilot，让出 can+pandaStates topic"""
    print("[setup] 1/3 stop comma.service ...")
    os.system("sudo systemctl stop comma 2>/dev/null")
    print("[setup] 2/3 kill 残留 pandad/manager ...")
    os.system("sudo pkill -9 -f 'selfdrive.pandad' 2>/dev/null")
    os.system("sudo pkill -9 -f 'manager.py' 2>/dev/null")
    time.sleep(2)
    print("[setup] 3/3 启动 manager（不带 pandad）...")
    os.system("cd /data/openpilot && nohup python3 system/manager/manager.py > /tmp/manager.log 2>&1 &")
    time.sleep(3)
    print("[setup] ✓ 完成。现在另起一终端跑 replay_can_csv.py <csv>")


def cleanup():
    """恢复真实 pandad"""
    print("[cleanup] kill manager + 重启 comma.service ...")
    os.system("sudo pkill -9 -f 'manager.py' 2>/dev/null")
    os.system("sudo systemctl start comma 2>/dev/null")
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

    if args.setup:   setup();   return
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
