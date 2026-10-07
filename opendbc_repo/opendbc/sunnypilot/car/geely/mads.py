"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Geely MADS模式实现 - 简化重构版
参考Honda、Hyundai等车型的实现

功能说明：
- MADS允许独立的横向控制激活/退出
- 支持基于ACC状态的自动激活
- 简洁的HUD反馈（白色/绿色/闪烁）
"""

from enum import StrEnum
from collections import namedtuple

from opendbc.car import Bus, DT_CTRL, structs
from opendbc.sunnypilot.mads_base import MadsCarStateBase
from opendbc.can.parser import CANParser

ButtonType = structs.CarState.ButtonEvent.Type

# MADS状态数据结构
MadsDataSP = namedtuple("MadsDataSP", ["enable_mads", "lat_active", "disengaging", "paused"])


class MadsCarController:
  """Geely MADS控制器 - 简化版本"""

  def __init__(self):
    super().__init__()
    self.mads = MadsDataSP(False, False, False, False)

    # 退出动画跟踪（参考Hyundai实现）
    self.lat_disengage_blink = 0
    self.lat_disengage_init = False
    self.prev_lat_active = False

    # HUD图标状态
    self.lkas_icon = 0

  def mads_status_update(self, CC: structs.CarControl, CC_SP: structs.CarControlSP, frame: int) -> MadsDataSP:
    """更新MADS状态 - 参考Hyundai的简洁实现"""
    enable_mads = CC_SP.mads.available

    # 检测横向控制下降沿，触发退出动画
    if CC.latActive:
      self.lat_disengage_init = False
    elif self.prev_lat_active:
      self.lat_disengage_init = True

    # 更新退出动画帧计数
    if not self.lat_disengage_init:
      self.lat_disengage_blink = frame

    # 计算状态
    paused = CC_SP.mads.enabled and not CC.latActive
    disengaging = (frame - self.lat_disengage_blink) * DT_CTRL < 1.0 if self.lat_disengage_init else False

    self.prev_lat_active = CC.latActive

    return MadsDataSP(enable_mads, CC.latActive, disengaging, paused)

  def create_lkas_icon(self, enabled: bool) -> int:
    """生成LKAS图标状态 - 参考Hyundai实现

    返回值：
    - 0: 关闭
    - 1: 可用（白色）
    - 2: 激活（绿色）
    - 3: 退出中（闪烁）
    """
    if self.mads.enable_mads:
      return 2 if self.mads.lat_active else 3 if self.mads.disengaging else 1
    else:
      return 2 if enabled else 1

  def update(self, CP: structs.CarParams, CC: structs.CarControl, CC_SP: structs.CarControlSP, frame: int) -> None:
    """更新MADS状态（每帧调用）"""
    self.mads = self.mads_status_update(CC, CC_SP, frame)
    self.lkas_icon = self.create_lkas_icon(CC.enabled)

class MadsCarState(MadsCarStateBase):
  """Geely MADS状态扩展 - 简化版本"""

  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    super().__init__(CP, CP_SP)
    self.main_cruise_enabled: bool = False

  @staticmethod
  def get_parser(CP, CP_SP, pt_messages) -> None:
    """添加MADS所需的CAN消息（Geely不需要额外消息）"""
    pass

  def get_main_cruise(self, ret: structs.CarState) -> bool:
    """获取ACC主开关状态

    Geely直接使用cruiseState.available作为主开关状态
    不支持手动切换（参考Honda的简单实现）
    """
    self.main_cruise_enabled = ret.cruiseState.available
    return self.main_cruise_enabled

  def update_mads(self, ret: structs.CarState, can_parsers: dict[StrEnum, CANParser]) -> None:
    """更新MADS状态（当前无需处理）

    Geely的MADS激活由Safety层自动处理：
    - 检测ACC_CMD.CRUISE_ENABLE上升沿
    - 自动设置横向控制允许标志

    如果未来添加LKAS按钮，可以在此处理按钮事件
    """
    pass
