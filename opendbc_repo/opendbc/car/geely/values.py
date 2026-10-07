"""Geely vehicle platform configuration and constants / 吉利车辆平台配置和常量"""
from dataclasses import dataclass, field
from enum import IntFlag

from opendbc.car import Bus, CarSpecs, PlatformConfig, Platforms
from opendbc.car.docs_definitions import CarDocs, CarParts, CarHarness
from opendbc.car.fw_query_definitions import FwQueryConfig, Request
from opendbc.car.structs import CarParams

Ecu = CarParams.Ecu

# Constants / 常量定义
HUD_MULTIPLIER = 1.035


class CarControllerParams:
  """Geely vehicle control parameters / 吉利车辆控制参数"""
  STEER_STEP = 2  # Steering command frequency: 100Hz / 2 = 50Hz / 转向命令频率

  # Steering torque limits / 转向力矩限制
  STEER_MAX = 598  # Verified maximum torque / 实车验证值

  # Steering rate limits / 转向速率限制
  STEER_DELTA_UP = 20      # Maximum increase per frame (@50Hz) / 每帧最大增加量
  STEER_DELTA_DOWN = 30    # Maximum decrease per frame (@50Hz) / 每帧最大减少量

  # Driver torque compensation must exactly match Panda's geely safety mode.
  # MAIN_TORQUE includes EPS load, not solely hand torque.  The former
  # allowance=0 / multiplier=15 / factor=2 made a harmless +59 reading clamp
  # a requested -598 command to zero, despite Panda allowing it.  Preserve a
  # real hand-override margin without treating EPS assistance as a driver.
  STEER_DRIVER_ALLOWANCE = 124
  STEER_DRIVER_MULTIPLIER = 1
  STEER_DRIVER_FACTOR = 1

  # Longitudinal control limits / 纵向控制限制
  ACCEL_MAX = 1.5      # Maximum acceleration (m/s²) / 最大加速度
  ACCEL_MIN = -3.5     # Maximum deceleration (m/s²) / 最大减速度

  def __init__(self, CP):
    # Read actual torque limits from lateralParams
    # 仌lateralParams读取实际扭矩限制
    # Ensure only one torque value (not speed-dependent)
    # 确保只有一个扭矩值（非速度依赖）
    lateral_params = getattr(CP, "lateralParams", None)
    if lateral_params and hasattr(lateral_params, "torqueV") and len(lateral_params.torqueV) > 0:
      self.STEER_MAX = int(lateral_params.torqueV[-1])

    # Adjust parameters based on vehicle flags / 根据车辆标志位调整参数
    if CP.flags & GeelyFlags.RAISED_ACCEL_LIMIT:
      self.ACCEL_MAX = 2.0


class GeelyFlags(IntFlag):
  """Geely vehicle feature flags / 吉利车型特性标志位"""
  RAISED_ACCEL_LIMIT = 1 << 0   # Raised acceleration limit / 提高加速度限制
  STOCK_ACC = 1 << 1            # Use stock ACC / 使用原车ACC
  ALC_ENABLED = 1 << 2          # Enable Automatic Lane Change / 启用自动变道
  MADS_ENABLED = 1 << 3         # MADS mode enabled / MADS模式启用


def dbc_dict(pt: str, radar: str | None = None, cam: str | None = None) -> dict:
  """Create DBC file mapping dictionary / 创建DBC文件映射字典"""
  result = {Bus.pt: pt}
  if radar is not None:
    result[Bus.radar] = radar
  if cam is not None:
    result[Bus.cam] = cam
  return result


@dataclass
class GeelyCarDocs(CarDocs):
  """Geely vehicle documentation configuration / 吉利车型文档配置"""
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.obd_ii]))


@dataclass
class GeelyPlatformConfig(PlatformConfig):
  """Geely platform configuration base class / 吉利平台配置基类"""
  dbc_dict: dict = field(default_factory=lambda: dbc_dict('geely_binyue_pt', radar='geely_radar_bus1', cam='geely_binyue_pt'))

  def __post_init__(self):
    """Platform initialization / 平台初始化"""
    super().__post_init__()
    self.flags |= GeelyFlags.ALC_ENABLED


class CAR(Platforms):
  """Supported Geely vehicle models / 吉利支持的车型枚举"""
  GEELY_BINYUE = GeelyPlatformConfig(
    [GeelyCarDocs("Geely Binyue (缤越)")],
    CarSpecs(
      mass=1370.0,
      wheelbase=2.6,
      # Fixed geometric baseline. Do not promote a persisted paramsd estimate
      # into this value: the learner is allowed to adapt from this starting
      # point per vehicle, while a stale cross-route estimate causes curve
      # oversteer and an activation kick.
      steerRatio=15.0,
      centerToFrontRatio=0.44,
      tireStiffnessFactor=0.9871
    ),
  )


# Create DBC mapping / 创建DBC映射
DBC = CAR.create_dbc_map()

# Firmware query configuration / 固件查询配置
FW_QUERY_CONFIG = FwQueryConfig(
  requests=[],
  # non_essential_ecus can be added later if needed
  # 非必需ECU可以后续添加
)

# Firmware version mapping / 固件版本映射
FW_VERSIONS = {
  "geely": [CAR.GEELY_BINYUE],
}

if __name__ == "__main__":
  cars = []
  for platform in CAR:
    for doc in platform.config.car_docs:
      cars.append(doc.name)
  cars.sort()
  for c in cars:
    print(c)
