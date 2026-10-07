"""吉利车辆接口。"""
from opendbc.car import Bus, structs, get_safety_config
from opendbc.car.interfaces import CarInterfaceBase
from opendbc.car.geely.carstate import CarState
from opendbc.car.geely.carcontroller import CarController
from opendbc.car.geely.radar_interface import RadarInterface
from opendbc.car.geely.values import CAR, DBC, CarControllerParams, GeelyFlags
from opendbc.safety import ALTERNATIVE_EXPERIENCE
from openpilot.common.params import Params


class CarInterface(CarInterfaceBase):
  """吉利车辆接口，支持传统模式和实验模式。"""

  # openpilot框架的类属性
  CarState = CarState
  CarController = CarController
  RadarInterface = RadarInterface

  @staticmethod
  def get_pid_accel_limits(CP, CP_SP, current_speed, cruise_speed):  # xiaomi8: upstream added CP_SP
    """获取纵向控制的PID加速度限制。"""
    return CarControllerParams(CP).ACCEL_MIN, CarControllerParams(CP).ACCEL_MAX

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, alpha_long, is_release, docs):
    """配置吉利车辆参数。"""
    ret.brand = "geely"
    ret.carFingerprint = candidate

    # 车辆规格
    ret.transmissionType = structs.CarParams.TransmissionType.automatic
    ret.radarUnavailable = Bus.radar not in DBC[candidate]

    # 转向配置
    ret.steerControlType = structs.CarParams.SteerControlType.torque
    # 转向延迟: 原为0.1s("参考Honda", 从未针对本车实测). 2026-08-29用真实行车数据实测
    # (OP下发角度命令 vs 实际测得方向盘角, 14段独立控车片段互相关, r=0.86~0.98):
    # 中位数200ms, 均值226ms, 范围140~420ms, 全部大于原假设值. 改为实测中位数.
    ret.steerActuatorDelay = 0.2   # 转向延迟 (2026-08-29实测, 原0.1参考Honda未经验证)
    ret.steerLimitTimer = 0.1      # 限制计时器

    # 横向调优
    ret.lateralTuning.init('pid')

    # 纵向控制模式 - 参考Honda Bosch处理方式
    # 支持实验模式：允许用户在UI中切换E2E纵向控制
    ret.alphaLongitudinalAvailable = True  # 启用实验性纵向控制选项
    ret.openpilotLongitudinalControl = alpha_long  # 用户是否启用了实验性纵向

    # 始终使用 pcmCruise 模式：直接读取 PCM_BUTTONS 的设定速度，而不是自己处理按钮
    # 这样 OP 和车机的定速会保持同步（都是每次 +1 km/h）
    ret.pcmCruise = True

    # 安全配置 - 根据纵向控制状态设置safety参数
    safety_param = 1 if ret.openpilotLongitudinalControl else 0
    ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.geely, safety_param)]

    # 禁用油门脱离ACC (踩油门时不退出巡航 - 吉利支持GAS_OVERRIDE模式)
    ret.alternativeExperience = ALTERNATIVE_EXPERIENCE.DEFAULT  # xiaomi8: DISABLE_DISENGAGE_ON_GAS removed upstream

    # MADS (M-Advanced Driver Assistance System) 模式
    # 开启后，按下ACC自动激活转向；关闭后，需要额外按LFA按钮激活转向
    params = Params()
    if params.get_bool("Mads"):
      ret.flags |= int(GeelyFlags.MADS_ENABLED)

    # 配置特定车辆参数
    if candidate == CAR.GEELY_BINYUE:
      # 转向扭矩限制
      ret.lateralParams.torqueBP = [0, 598]
      ret.lateralParams.torqueV = [0, 598]

      # 横向控制PID调优 - 对齐Proton参数
      ret.lateralTuning.pid.kpBP = [0., 25., 35., 40.]
      ret.lateralTuning.pid.kpV = [0.06, 0.18, 0.18, 0.19]  # 使用Proton的P增益
      ret.lateralTuning.pid.kiBP = [0., 20., 30.]
      ret.lateralTuning.pid.kiV = [0.04, 0.08, 0.16]  # 使用Proton的I增益
      # SP2025 validated this value on the Mi 8/V4L2 path. The later 1.5e-6
      # Proton value overdrives both left and right curves while straight-line
      # behavior remains normal, so keep the validated Geely feedforward.
      ret.lateralTuning.pid.kf = 0.0000008

      # 吉利 ACC 自身已有执行器闭环；不能直接套用 Proton 的高增益。
      # 高增益会把 aEgo 的微小测量/执行器波动放大为油门-刹车循环，
      # 尤其在有前车时表现为反复拉锯。此表来自已验证的 Geely 标定分支。
      ret.longitudinalTuning.kpBP = [0., 5., 20.]
      ret.longitudinalTuning.kpV = [1.2, 0.8, 0.6]
      ret.longitudinalTuning.kiBP = [0., 5., 20.]
      ret.longitudinalTuning.kiV = [0.18, 0.12, 0.08]
      ret.longitudinalActuatorDelay = 0.5

      # 车辆动力学 - wheelbase/centerToFront/minEnableSpeed 等由 CarSpecs 自动设置
      ret.wheelSpeedFactor = 1.0

      # 启用功能
      ret.enableBsm = True  # 盲区监测

      # 低速和停车参数（关键：避免MPC在低速时失败）
      ret.stopAccel = -2.0          # 停车时的加速度 (m/s²)
      # xiaomi8 fix: schema 无此字段, 注释掉
      # ret.vEgoStopping = ...
      # xiaomi8 fix: schema 无此字段, 注释掉
      # ret.vEgoStarting = ...
      # xiaomi8 fix: schema 无此字段, 注释掉
      # ret.stoppingDecelRate = ...

      # 停止-继续自动恢复 - 仅在OP纵向控制启用时
      ret.autoResumeSng = ret.openpilotLongitudinalControl

    else:
      # 未知车辆 - 仅仪表盘摄像头
      ret.dashcamOnly = True
      ret.safetyConfigs = [get_safety_config(structs.CarParams.SafetyModel.noOutput)]

    return ret

  # 接口的其余部分使用基类实现
  # CarState和CarController将自动导入
