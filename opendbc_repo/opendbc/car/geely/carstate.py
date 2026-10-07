"""吉利车辆状态监控。"""
import copy
import numpy as np
from opendbc.can import CANParser, CANDefine
from opendbc.car import Bus, structs, create_button_events
from opendbc.car.interfaces import CarStateBase
from opendbc.car.geely.values import DBC, HUD_MULTIPLIER
from opendbc.car.common.conversions import Conversions as CV
from opendbc.sunnypilot.car.geely.mads import MadsCarState

ButtonType = structs.CarState.ButtonEvent.Type


class CarStateConstants:
  """吉利车辆状态常量。"""
  STEERING_TORQUE_THRESHOLD = 50   # 驾驶员扭矩阈值 (降低以支持轻触检测)
  GAS_PEDAL_THRESHOLD = 0.01       # 油门踏板阈值
  BLINKER_MIN_FRAMES = 112         # 转向灯最小持续帧数 (2.24秒 @ 50Hz)
  STANDSTILL_THRESHOLD = 0.01      # 静止速度阈值

  # 按钮编码值
  BUTTON_CANCEL = 3
  BUTTON_SET_DECEL = 5
  BUTTON_SET_ACTIVATE = 1
  BUTTON_RES_ACCEL = 4
  BUTTON_RES_RESUME = 2


class CarState(CarStateBase, MadsCarState):
  """吉利车辆状态监控和CAN信号解析。"""

  def __init__(self, CP, CP_SP=None):
    CarStateBase.__init__(self, CP, CP_SP)
    MadsCarState.__init__(self, CP, CP_SP)

    # 使用 CANDefine 获取 DBC 定义的值映射
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.shifter_values = can_define.dv["TRANSMISSION"]["GEAR"]

    self.cruise_enable = False
    self.stock_counter = 0
    self.prev_angle = 0  # 用于计算转向方向

    # 按钮状态跟踪
    self.prev_cruise_buttons = 0
    self.prev_distance_button = 0
    self.distance_button = 0

    # 车道保持辅助增强
    self.left_lane_visible = False
    self.right_lane_visible = False
    self.left_lane_depart = False
    self.right_lane_depart = False
    self.ldw_alert = False

    # 前车检测 - 从ADAS_LEAD_DETECT (0x1A6=422)读取
    self.lead_distance = 0.0  # 前车距离(米)
    self.lead_visible = False  # 前车是否可见

    # 盲区监测增强
    self.left_bsm_warning = False
    self.right_bsm_warning = False
    self.bsm_active = False
    # 原车摄像头融合信号（方案1：SNG增强）
    self.camera_lead_distance = 0  # 原车摄像头检测的前车距离 (0-255)
    self.camera_lead_detected = False  # 原车是否检测到前车
    self.camera_lead1 = False  # 前车1
    self.camera_lead2 = False  # 前车2
    self.camera_lead_too_near = False  # 前车过近预警

    # 原车摄像头车道车辆检测（方案2：盲区增强）
    self.camera_left_lane_car_dist = 0  # 左车道车辆距离 (0-255)
    self.camera_left_lane_car_exist = False  # 左车道有车
    self.camera_right_lane_car_dist = 0  # 右车道车辆距离 (0-255)
    self.camera_right_lane_car_exist = False  # 右车道有车
    # 存储原始摄像头消息用于拦截
    self.cam_lkas_msg = {}  # ADAS_LKAS (0x1B0)
    self.cam_acc_msg = {}  # ACC_CMD (0x1A1)

  def _safe_get_vl(self, parser, message_name, default=None):
    """安全获取CAN消息值，避免KeyError。"""
    return parser.vl.get(message_name, default or {})

  def _update_vehicle_dynamics(self, cp, ret):
    """更新车辆动力学状态（速度、转向、踏板、档位）。"""
    # 车轮速度 (xiaomi8: upstream parse_wheel_speeds signature changed — now takes cs as first arg, sets vEgoRaw/vEgo/aEgo in-place)
    self.parse_wheel_speeds(ret,
      cp.vl["WHEEL_SPEED"]["WHEELSPEED_F"],
      cp.vl["WHEEL_SPEED"]["WHEELSPEED_F"],
      cp.vl["WHEEL_SPEED"]["WHEELSPEED_B"],
      cp.vl["WHEEL_SPEED"]["WHEELSPEED_B"],
    )
    ret.standstill = ret.vEgoRaw < CarStateConstants.STANDSTILL_THRESHOLD
    ret.vEgoCluster = ret.vEgo * HUD_MULTIPLIER

    # 转向 - 直接使用原始角度（与SP保持一致，绝对编码器不需要offset学习）
    ret.steeringAngleDeg = cp.vl["STEERING_MODULE"]["STEER_ANGLE"]

    # 计算转向方向（用于扭矩符号）
    steer_dir = 1 if (ret.steeringAngleDeg - self.prev_angle >= 0) else -1
    self.prev_angle = ret.steeringAngleDeg

    # 驾驶员扭矩 - 使用MAIN_TORQUE
    # MAIN_TORQUE: 10位(0-1023)，无符号，包含驾驶员和EPS的总扭矩
    # DRIVER_TORQUE_NO_ADAS: 5位(0-31)，范围太小不可用
    # 通过转向角变化方向(steer_dir)添加符号，与safety_geely.h保持一致
    ret.steeringTorque = cp.vl["STEERING_TORQUE"]["MAIN_TORQUE"] * steer_dir
    ret.steeringTorqueEps = cp.vl["STEERING_MODULE"]["STEER_RATE"] * steer_dir

    # 检测方向盘接管 - 阈值根据MAIN_TORQUE范围调整
    ret.steeringPressed = abs(ret.steeringTorque) > 124

    # 踏板 - 参考Honda/Toyota实现
    # DBC定义: APPS_1 scale=0.004, 范围[0-255]*0.004 = [0-1.02]
    # xiaomi8: upstream removed CarState.gas (float); only gasPressed (bool) remains
    gas_pedal = cp.vl["GAS_PEDAL"]["APPS_1"]
    ret.gasPressed = min(gas_pedal, 1.0) > CarStateConstants.GAS_PEDAL_THRESHOLD
    # xiaomi8 fix: upstream 删除了 CarState.brake (float), 只剩 brakePressed (bool), 见下
    ret.brakePressed = bool(cp.vl["PARKING_BRAKE"]["BRAKE_PRESSED"])
    ret.brakeHoldActive = bool(cp.vl["PARKING_BRAKE"]["CAR_ON_HOLD"])
    ret.parkingBrake = bool(cp.vl["PARKING_BRAKE"]["ENGAGING_TILL_RELEASE"])

    # 档位
    can_gear = int(cp.vl["TRANSMISSION"]["GEAR"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

  def _update_safety_systems(self, cp, cp_cam, ret):
    """更新安全系统状态（车门、安全带、转向灯、盲区监测）。"""
    # 车门和安全带
    ret.doorOpen = any([
      cp.vl["DOOR_LEFT_SIDE"]["FRONT_LEFT_DOOR"],
      cp.vl["DOOR_LEFT_SIDE"]["BACK_LEFT_DOOR"],
      cp.vl["DOOR_RIGHT_SIDE"]["FRONT_RIGHT_DOOR"],
      cp.vl["DOOR_RIGHT_SIDE"]["BACK_RIGHT_DOOR"],
    ])
    ret.seatbeltUnlatched = not bool(cp.vl["SEATBELTS"]["LEFT_SIDE_SEATBELT_ACTIVE_LOW"])

    # 转向灯
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_stalk(
      CarStateConstants.BLINKER_MIN_FRAMES,
      cp.vl["LEFT_STALK"]["LEFT_SIGNAL"],
      cp.vl["LEFT_STALK"]["RIGHT_SIGNAL"])

    # 盲区监测增强
    if self.CP.enableBsm:
      self._update_blind_spot_monitoring(cp, ret)

    # 安全检查
    ret.stockAeb = False
    ret.stockFcw = bool(cp.vl["FCW"]["STOCK_FCW_TRIGGERED"])
    ret.espDisabled = not bool(cp.vl["PARKING_BRAKE"]["ESC_ON"])

    # 转向故障检测 - 保守策略（参考Honda）
    # 不因正常过弯扭矩大而退出，只在真正严重故障时才标记
    # 由于Geely DBC中无明确的EPS故障信号，暂时保持False（允许继续控制）
    # 如果发现实车有EPS故障位，可以在此处添加检测
    ret.steerFaultTemporary = False  # 保守：不轻易退出控制
    ret.steerFaultPermanent = False

    # 额外信号
    ret.steeringRateDeg = cp.vl["STEERING_MODULE"]["STEER_RATE"]
    ret.genericToggle = bool(cp.vl["LEFT_STALK"]["GENERIC_TOGGLE"])

  def _update_camera_fusion_signals(self, cp_cam, ret):
    """解析原车摄像头融合信号（方案1+2）。

    Args:
      cp_cam: CAM总线解析器
      ret: CarState.CarStateData对象
    """
    # 【方案1：SNG增强】解析ADAS_LEAD_DETECT (0x1A6)
    self.camera_lead_distance = int(cp_cam.vl["ADAS_LEAD_DETECT"]["LEAD_DISTANCE"])
    self.camera_lead1 = bool(cp_cam.vl["ADAS_LEAD_DETECT"]["IS_LEAD1"])
    self.camera_lead2 = bool(cp_cam.vl["ADAS_LEAD_DETECT"]["IS_LEAD2"])
    self.camera_lead_too_near = bool(cp_cam.vl["ADAS_LEAD_DETECT"]["LEAD_TOO_NEAR"])
    self.camera_lead_detected = self.camera_lead1 or self.camera_lead2

    # 融合原车摄像头和openpilot的前车检测
    # 如果原车检测到且距离合理(5-200米)，可用于SNG增强
    if self.camera_lead_detected and 5 <= self.camera_lead_distance <= 200:
      # 保存到CarState用于SNG逻辑
      self.lead_distance = float(self.camera_lead_distance)
      self.lead_visible = True

    # 【方案2：盲区增强】解析ADAS_CAR_DETECT (0x1A7)
    self.camera_left_lane_car_dist = int(cp_cam.vl["ADAS_CAR_DETECT"]["LEFT_LANE_CAR_DIST"])
    self.camera_left_lane_car_exist = bool(cp_cam.vl["ADAS_CAR_DETECT"]["LEFT_LANE_CAR_EXIST"])
    self.camera_right_lane_car_dist = int(cp_cam.vl["ADAS_CAR_DETECT"]["RIGHT_LANE_CAR_DIST"])
    self.camera_right_lane_car_exist = bool(cp_cam.vl["ADAS_CAR_DETECT"]["RIGHT_LANE_CAR_EXIST"])

  def _update_blind_spot_monitoring(self, cp, ret):
    """更新盲区监测状态（方案2：集成车道车辆检测）。"""
    left_approach = bool(cp.vl["BSM_ADAS"]["LEFT_APPROACH"])
    left_warning = bool(cp.vl["BSM_ADAS"]["LEFT_APPROACH_WARNING"])
    right_approach = bool(cp.vl["BSM_ADAS"]["RIGHT_APPROACH"])
    right_warning = bool(cp.vl["BSM_ADAS"]["RIGHT_APPROACH_WARNING"])

    # 【方案2增强】结合原车摄像头车道车辆检测
    # BSM主要检测后方盲区，摄像头可以检测侧前方车辆
    # 综合两者提供更全面的侧方感知
    left_lane_car = self.camera_left_lane_car_exist
    right_lane_car = self.camera_right_lane_car_exist

    # 分级警告逻辑：BSM或摄像头任一检测到即标记
    ret.leftBlindspot = left_approach or left_warning or left_lane_car
    ret.rightBlindspot = right_approach or right_warning or right_lane_car

    # 保存详细状态用于HUD显示
    self.left_bsm_warning = left_warning
    self.right_bsm_warning = right_warning
    self.bsm_active = left_approach or right_approach or left_warning or right_warning or left_lane_car or right_lane_car

    # 增强安全检测：转向灯+盲区同时激活时加强警告
    if ret.leftBlinker and (left_warning or left_lane_car):
      ret.leftBlindspot = True
    if ret.rightBlinker and (right_warning or right_lane_car):
      ret.rightBlindspot = True

  def _update_driver_assistance(self, cp_cam, ret):
    """更新驾驶辅助状态（车道保持、前车检测）。"""
    # 车道保持辅助视觉状态
    self.left_lane_visible = not bool(cp_cam.vl["LKAS"]["LEFT_LANE_VISIBLE_DISENGAGE"])
    self.right_lane_visible = not bool(cp_cam.vl["LKAS"]["RIGHT_LANE_VISIBLE_DISENGAGE"])

    # 车道偏离检测
    self.left_lane_depart = bool(cp_cam.vl["LKAS"]["LANE_DEPARTURE_AUDIO_LEFT"])
    self.right_lane_depart = bool(cp_cam.vl["LKAS"]["LANE_DEPARTURE_AUDIO_RIGHT"])
    self.ldw_alert = self.left_lane_depart or self.right_lane_depart

    # 前车检测
    lead1_visible = bool(cp_cam.vl["ADAS_LEAD_DETECT"]["IS_LEAD1"])
    lead2_visible = bool(cp_cam.vl["ADAS_LEAD_DETECT"]["IS_LEAD2"])
    self.lead_visible = lead1_visible or lead2_visible
    self.lead_distance = float(cp_cam.vl["ADAS_LEAD_DETECT"]["LEAD_DISTANCE"]) if self.lead_visible else 0.0

  def _update_cruise_control(self, cp_cam, ret):
    """更新巡航控制状态"""
    # 从摄像头总线读取巡航控制状态
    self.cruise_enable = bool(cp_cam.vl["ACC_CMD"]["CRUISE_ENABLE"])
    acc_req = bool(cp_cam.vl["ACC_CMD"]["ACC_REQ"])
    acc_not_req = bool(cp_cam.vl["ACC_CMD"]["NOT_ACC_REQ"])

    # 读取油门覆盖状态
    pcm_buttons_vl = self._safe_get_vl(cp_cam, "PCM_BUTTONS")
    self.gas_override = gas_override = bool(pcm_buttons_vl.get("GAS_OVERRIDE", False)) if pcm_buttons_vl else False

    # 从CAM总线获取原始消息用于转发
    self.cam_lkas_msg = copy.copy(cp_cam.vl["ADAS_LKAS"])
    self.cam_lkas_hud_msg = copy.copy(cp_cam.vl["LKAS"])
    self.cam_acc_msg = copy.copy(cp_cam.vl["ACC_CMD"])

    # ACC激活逻辑
    stock_acc_active = acc_req and not acc_not_req

    # cruiseState.available状态
    cruise_available = self.cruise_enable or gas_override or stock_acc_active
    ret.cruiseState.available = cruise_available

    # 获取standstill状态
    acc_cmd_vl = self._safe_get_vl(cp_cam, "ACC_CMD")
    cruise_standstill = bool(acc_cmd_vl.get("STANDSTILL2", False))

    # 设置cruiseState（参考Honda实现）
    # available：ACC主开关状态
    # enabled：原厂ACC实际激活状态
    ret.cruiseState.available = cruise_available  # ACC开关打开或原厂ACC激活
    ret.cruiseState.enabled = stock_acc_active    # 只有原厂ACC激活时才为true
    ret.cruiseState.standstill = cruise_standstill

  def _update_button_events(self, cp, cp_cam, ret):
    """更新按钮事件。"""
    # 注释掉按钮处理：使用 pcmCruise 模式，直接读取车机的设定速度
    # 这样 OP 和车机的定速会保持同步（都是每次 +1 km/h）
    # cruise_buttons = self._decode_cruise_buttons(cp)
    # buttonEvents = self._create_button_events(cruise_buttons)
    # self.prev_cruise_buttons = cruise_buttons

    # 巡航速度设置 - 直接从 PCM_BUTTONS 读取车机的设定速度
    pcm_buttons_vl = self._safe_get_vl(cp_cam, "PCM_BUTTONS")
    if pcm_buttons_vl:
      cruise_speed_kmh = int(pcm_buttons_vl.get("ACC_SET_SPEED", 0))
      cruise_speed_ms = cruise_speed_kmh * CV.KPH_TO_MS

      # 设置巡航速度到所有相关字段（确保实时更新）
      ret.cruiseState.speed = cruise_speed_ms / HUD_MULTIPLIER
      ret.cruiseState.speedCluster = cruise_speed_ms
      ret.vCruise = cruise_speed_kmh  # km/h 用于纵向控制PID
      ret.vCruiseCluster = cruise_speed_kmh  # km/h 用于UI显示

      # 检测距离按钮按下 - 创建gapAdjustCruise按钮事件用于切换驾驶模式
      prev_distance_button = self.distance_button
      self.distance_button = 1 if bool(pcm_buttons_vl.get("DISTANCE_SETTING_BUTTON", False)) else 0

      # 创建gapAdjustCruise按钮事件
      ret.buttonEvents = create_button_events(self.distance_button, prev_distance_button, {1: ButtonType.gapAdjustCruise})

    else:
      ret.buttonEvents = []

  def _decode_cruise_buttons(self, cp):
    """解码巡航按钮状态，返回标准化的按钮值。"""
    acc_buttons_vl = self._safe_get_vl(cp, "ACC_BUTTONS")
    if not acc_buttons_vl:
      return 0

    button_set = bool(acc_buttons_vl.get("SET_BUTTON", False))
    button_res = bool(acc_buttons_vl.get("RES_BUTTON", False))
    cruise_btn = bool(acc_buttons_vl.get("CRUISE_BTN", False))

    # 按优先级编码按钮状态
    if cruise_btn:
      return CarStateConstants.BUTTON_CANCEL
    elif button_set:
      return CarStateConstants.BUTTON_SET_DECEL if self.cruise_enable else CarStateConstants.BUTTON_SET_ACTIVATE
    elif button_res:
      return CarStateConstants.BUTTON_RES_ACCEL if self.cruise_enable else CarStateConstants.BUTTON_RES_RESUME

    return 0

  def _create_button_events(self, cruise_buttons):
    """根据按钮值创建按钮事件列表。"""
    button_events_dict = {
      CarStateConstants.BUTTON_SET_ACTIVATE: ButtonType.accelCruise,      # SET (设置当前速度)
      CarStateConstants.BUTTON_RES_RESUME: ButtonType.resumeCruise,       # RES (恢复速度)
      CarStateConstants.BUTTON_CANCEL: ButtonType.cancel,                 # CANCEL (取消巡航)
      CarStateConstants.BUTTON_RES_ACCEL: ButtonType.accelCruise,         # RES加速
      CarStateConstants.BUTTON_SET_DECEL: ButtonType.decelCruise,         # SET减速
    }

    return create_button_events(cruise_buttons, self.prev_cruise_buttons, button_events_dict)

  def update(self, can_parsers, hud_control=None):
    """从CAN消息更新车辆状态。

    参数:
      can_parsers: 不同总线的CAN解析器字典

    返回:
      tuple: (CarState, CarStateSP) - 主状态和SunnyPilot扩展
    """
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    ret = structs.CarState()

    # 更新车辆动力学状态
    self._update_vehicle_dynamics(cp, ret)

    # 解析原车摄像头融合信号（方案1+2）
    self._update_camera_fusion_signals(cp_cam, ret)

    # 更新安全系统状态
    self._update_safety_systems(cp, cp_cam, ret)

    # 更新驾驶辅助状态
    self._update_driver_assistance(cp_cam, ret)

    # 更新巡航控制状态
    self._update_cruise_control(cp_cam, ret)

    # 更新按钮事件
    self._update_button_events(cp, cp_cam, ret)

    # MADS状态更新(Geely无独立LKAS按钮, 当前为no-op, 由Safety层自动处理)
    MadsCarState.update_mads(self, ret, can_parsers)

    ret_sp = structs.CarStateSP()
    return ret, ret_sp  # xiaomi8: upstream now expects tuple

  def get_can_parsers(self, CP, CP_SP=None):
    """为吉利车辆创建CAN消息解析器。

    根据车辆配置动态订阅所需的CAN消息。
    """
    # PT总线 (Bus 0) - 基本车辆状态消息（所有车型必需）
    pt_messages = [
      ("STEERING_MODULE", 100),
      ("STEERING_TORQUE", 50),
      ("WHEEL_SPEED", 50),
      ("BRAKE", 50),
      ("GAS_PEDAL", 50),
      ("ENGINE", 50),
      ("TRANSMISSION", 50),
      ("PARKING_BRAKE", 50),
      ("SEATBELTS", 10),
      ("DOOR_LEFT_SIDE", 10),
      ("DOOR_RIGHT_SIDE", 10),
      ("LEFT_STALK", 10),
      ("POWER_MODE", 10),
      ("FCW", 50),
      ("ACC_BUTTONS", 50),
    ]

    # 盲区监测 (BSM) - 可选配置
    if CP.enableBsm:
      pt_messages.append(("BSM_ADAS", 50))

    # CAM总线 (Bus 2) - 来自原车摄像头的ADAS消息
    cam_messages = [
      ("ACC_CMD", 50),       # ACC纵向控制命令
      ("ADAS_LKAS", 50),     # LKAS转向控制命令
      ("LKAS", 50),          # 车道线状态/HUD
      ("PCM_BUTTONS", 50),   # 巡航设置速度
    ]

    # 前车检测和车道车辆检测 - 用于雷达接口
    # 这些消息由原车摄像头发送，包含前车距离等信息
    cam_messages += [
      ("ADAS_LEAD_DETECT", 50),  # 前车检测 (0x1A6)
      ("ADAS_CAR_DETECT", 50),   # 车道车辆检测 (0x1A7)
    ]

    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], pt_messages, 0),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.cam], cam_messages, 2),
    }
