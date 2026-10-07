"""Geely 缤越 — Continental ARS410(ARS4-B) 前雷达接口 (bus1 雷达直出)

═══ 数据来源 ═══
  Bus.radar (bus1)  雷达直出, 未经融合:
     RADAR_TRACK_A_xx (0x20+k)  距离/相对速度/横向速度/计数器/CRC
     RADAR_TRACK_B_xx (0x50+k)  航迹ID+代次/动静类/相对加速度/sigma/计数器/CRC
     FRS_STATUS (0x080)         车->雷达 本车速度+偏航(雷达做静止物补偿用)

═══ 逆向依据 (2026-07-19/21, 车型/123 全量CSV 700万+报文, 详见 geely_ARS410_逆向总结.md) ═══
  ★2026-07-21 实车路测 277万帧/1958s 全面验证 (真值链均可追溯到轮速, 非传感器互比)★
  距离   DISTANCE  = 0.98*(b3<<8|b4)/256 - 3.4
         运动学自洽(vRel≡d(dRel)/dt, 半窗12帧+双向定界): 全部 r=0.9991 s∈[0.984,0.986];
         ★移动目标 r=0.9988 s∈[0.993,0.996]; 快速目标 s∈[0.995,0.997]★ 残差 0.29~0.33 m/s
         注: raw=0 是无效标记(解码得 -3.40m), raw<1018 全属物理不可能, 由 MIN_DIST 挡掉(占2.05%)
  速度   REL_SPEED = (b0-127.6)/1.27  [负=接近]
         静止物对地速度≡0(真值=轮速): 50万样本重心 -0.051 m/s; 分5个速度档重心 0.000~-0.067,
         残余斜率仅 -0.4%/(m/s) —— 且 0x080 自报车速 vs 轮速本就差 0.64%(轮胎标定), 雷达可能更准
  加速度 REL_ACCEL = (g2b4-128)/12.5   静止物 aRel≡-aEgo: n=38159 r=0.898 双向斜率[0.958,1.188]
  横向速 LAT_SPEED = (b5-128)/1.27     真值 -w*dRel (w取自0x080, 不依赖steerRatio): r=0.994,
         零点实测 128.00(14.9万条直行样本 std 0.575); scale 随距离漂是真值模型没含方位角所致,
         趋势收敛到 ~1.27~1.30 => 横纵共用同一量化
  动静类 g2.b3: ★单向蕴含★ 30~60 ⇒ 静止 可靠(误判率2.0%); ≥100 ⇒ 移动 仅64%可靠, 且雷达要
         目标>6m/s 才肯标"移动", 1~6m/s 慢车仍编在30~60区 ⇒ 【不可反用】. 本文件已不再使用该字段.
  航迹ID g2.b0 = 槽号 + 40*代次n; 槽被新目标复用时 n 必变(实测99.7%), 同目标连续时不变(0.4%)
         => (槽号,n) 是雷达自己的航迹ID
  横向位置 LAT_POS = g1.byte1[4:0]+byte2 BE(12|13) *0.015625 -64  [负右/正左, m]
         ★2026-08-20 订正: 早先"横向不存在"错了(几何法把纵向XPos当径向R造错真值, 见逆向总结)★
         官方 Continental FRS DBC(车型/geely_ars410_radar.dbc) 有 YPos, 用实车4重验证:
         ①前车YPos≈0 ②静止物闭合率各|YPos|档恒=满vEgo(证 byte3-4=纵向非径向)
         ③d(YPos)/dt vs YVelRel(byte5,已知横向速度) 斜率1.00 ④真CANParser解码逐帧一致
         => yRel = LAT_POS. radard 用它做视觉-雷达横向匹配(同距离时区分在道前车 vs 路边物)

═══ 架构 (2026-07-21 重写) ═══
  ★本接口只做"解码 + 航迹管理", 不做前车选择★

  radard 的契约 (selfdrive/controls/radard.py):
     ar_pts = {pt.trackId: [dRel, yRel, vRel, measured] for pt in rr.points}
     for ids in list(self.tracks):
         if ids not in ar_pts: self.tracks.pop(ids)      # ← 本帧缺席=立即销毁
     ...
     match_vision_to_track(): track = max(tracks.values(), key=prob)   # ← 在全部航迹里挑

  即 radard 要"全部航迹", 由它自己拿 modelV2.leadsV3 做视觉-雷达匹配.

  旧版(已废弃)在这里自己先用吉利摄像头 0x1A6 做了一遍融合, 只吐 1 个点, 后果:
    1. 出点要同时满足 三个AND (吉利摄像头看见 ∧ op模型看见 ∧ 单点落在 max(0.25d,5m) 窗口内),
       任一不满足 -> match_vision_to_track 返回 None -> leadOne.radar=False -> 退回纯视觉 -> 刹不及时
    2. max(tracks) 只有一个候选, 选错了没有备选
    3. 摄像头没确认时 ret.points=[] -> radard 把所有 Track 清空 -> 卡尔曼滤波器反复重建

  路边杂波不用在这里滤: radard 的 prob_v 项已经区分 —— 静止物 vRel+vEgo≈0, 前车≈vEgo,
  速度似然直接把护栏/路牌拉开.

  ★寿命过滤按距离分级 (2026-07-21 实车修正, 见 NEAR_DIST)★
  最初一刀切 MIN_TRACK_FRAMES=6 压杂波, 但实测 <15m 目标航迹寿命【中位仅2周期】, 59%活不到6,
  导致近距离目标从不输出 => 越靠近前车越容易丢 => 溜车不停. 现改为近处不过滤.
"""
from opendbc.can import CANParser
from opendbc.car import Bus, structs
from opendbc.car.interfaces import RadarInterfaceBase
from opendbc.car.geely.values import DBC

RADAR_A_START = 0x20      # RADAR_TRACK_A_00..39
RADAR_B_START = 0x50      # RADAR_TRACK_B_00..39
N_SLOTS = 40
TRIGGER_MSG = RADAR_B_START + N_SLOTS - 1   # 0x77, 一轮里最后一条 -> 用作触发

MIN_DIST, MAX_DIST = 0.5, 200.0
TRACK_TIMEOUT = 20        # 超过此周期未更新则清除航迹
EMPTY_ID = 0xFF           # 空槽标记. 注意 0xFF%40==15, 光靠槽号自洽校验挡不住, 必须显式判

# ★★航迹寿命门槛 —— 必须按距离分级★★ (2026-07-21 实车 277万帧修正)
#   实测: <15m 目标的连续存活周期数【中位仅 2】, 59% 活不到 6 周期.
#   原来一刀切 MIN_TRACK_FRAMES=6 => 59% 的近距离目标【从未被输出给 radard】
#   => 越靠近前车航迹越碎 => radard 每帧 tracks.pop() 清空卡尔曼 => 规划器认为前方无车
#   => 松刹车继续往前溜. 这正是"识别到只有几米也减速了, 但到1~2米还在走"的根因.
#   近处宁可误报(轻微点刹, 可忍受)也不能漏报(撞上去). 远处才用寿命过滤压杂波.
NEAR_DIST = 30.0          # 此距离内不做寿命过滤, 见到就报
MIN_TRACK_FRAMES_FAR = 6  # 远处航迹最少存活周期(~0.3s) — 挡短命杂波


def _create_parsers(car_fingerprint):
  radar_msgs = []
  for k in range(N_SLOTS):
    radar_msgs.append((f"RADAR_TRACK_A_{k:02d}", 15))
    radar_msgs.append((f"RADAR_TRACK_B_{k:02d}", 15))
  return CANParser(DBC[car_fingerprint][Bus.radar], radar_msgs, 1)


class RadarInterface(RadarInterfaceBase):
  def __init__(self, CP, CP_SP=None):
    super().__init__(CP, CP_SP)
    self.rcp = _create_parsers(CP.carFingerprint)
    self.trigger_msg = TRIGGER_MSG
    self.updated_messages = set()

    self.track_id = 0
    self._tracks = {}        # (slot, gen) -> [trackId, first_cycle, last_cycle]
    self._cycle = 0

  def update(self, can_strings):
    if self.rcp is None:
      return super().update(None)

    self.updated_messages.update(self.rcp.update(can_strings))

    # 等一轮雷达报文到齐再处理 (返回 None 表示本次无新数据 — op 接口约定)
    if self.trigger_msg not in self.updated_messages:
      return None

    ret = self._update()
    self.updated_messages.clear()
    return ret

  def _update(self):
    ret = structs.RadarData()
    if not self.rcp.can_valid:
      ret.errors.canError = True
      return ret
    self._cycle += 1

    seen = set()
    for k in range(N_SLOTS):
      a = self.rcp.vl[f"RADAR_TRACK_A_{k:02d}"]
      b = self.rcp.vl[f"RADAR_TRACK_B_{k:02d}"]

      raw_id = int(b["TRACK_ID_GEN"])
      if raw_id == EMPTY_ID or raw_id % N_SLOTS != k:   # 空槽 / 槽号自洽校验(实测100%)
        continue
      # 注: raw=0 是无效标记, 解码得 -3.40m; raw<1018 全部 <0.5m 属物理不可能,
      #     一律被下面的 MIN_DIST 挡掉(实测占 2.05%), 不需另设判据.
      drel = float(a["DISTANCE"])
      if not (MIN_DIST < drel < MAX_DIST):
        continue

      key = (k, raw_id // N_SLOTS)        # ★(槽号, 代次) = 雷达自己的航迹ID
      tr = self._tracks.get(key)
      if tr is None:
        tr = self._tracks[key] = [self.track_id, self._cycle, self._cycle]
        self.track_id += 1
      else:
        tr[2] = self._cycle
      # ★寿命过滤只对远处生效 —— 近处见到就报, 漏报比误报危险得多
      if drel >= NEAR_DIST and self._cycle - tr[1] < MIN_TRACK_FRAMES_FAR:
        continue

      tid = tr[0]
      seen.add(tid)
      pt = self.pts.get(tid)
      if pt is None:
        pt = self.pts[tid] = structs.RadarData.RadarPoint()
        pt.trackId = tid                  # ★radard 按 trackId 索引卡尔曼滤波器, 必须稳定
        pt.deprecated.measured = True
      yrel = float(a["LAT_POS"])
      # The ARS410 encodes an unavailable lateral position as raw 0, which
      # decodes to exactly -64.0 m. Real bus1 replay contains this sentinel
      # paired with impossible high closing speeds; it is not a target at the
      # edge of the radar FOV. Do not feed it to radard's vision matcher.
      if yrel <= -63.9:
        continue

      pt.dRel = drel
      pt.yRel = yrel                         # ★横向位置(见文件头), radard 靠它做横向匹配
      pt.vRel = float(a["REL_SPEED"])

    for tid in [t for t in self.pts if t not in seen]:
      del self.pts[tid]
    for key, v in list(self._tracks.items()):
      if self._cycle - v[2] > TRACK_TIMEOUT:
        del self._tracks[key]

    ret.points = list(self.pts.values())
    return ret
