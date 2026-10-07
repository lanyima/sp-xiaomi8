"""吉利CAN消息创建工具。"""

# CRC8校验和计算（多项式0x2F，初始化0xFF，异或输出0xFF）
_CRC8_POLY = 0x2F
_CRC8_INIT = 0xFF
_CRC8_XOROUT = 0xFF
_CRC8_TABLE = None


def _init_crc8_table():
  """初始化CRC8查找表以加快校验和计算。"""
  global _CRC8_TABLE
  if _CRC8_TABLE is None:
    table = []
    for value in range(256):
      crc = value
      for _ in range(8):
        if crc & 0x80:
          crc = ((crc << 1) ^ _CRC8_POLY) & 0xFF
        else:
          crc = (crc << 1) & 0xFF
      table.append(crc)
    _CRC8_TABLE = table
  return _CRC8_TABLE


def _crc8(data):
  """计算给定数据的CRC8校验和。"""
  table = _init_crc8_table()
  crc = _CRC8_INIT
  for byte in data:
    crc = table[crc ^ byte]
  return crc ^ _CRC8_XOROUT


def _copy_cam_msg_with_defaults(cam_msg, defaults):
  """复制摄像头消息，使用默认值填充缺失字段。"""
  return {s: cam_msg.get(s, defaults[s]) for s in defaults.keys()}


def _make_can_msg_with_checksum(packer, msg_name, values, checksum_field="CHECKSUM"):
  """通用CAN消息创建函数，自动计算校验和。

  所有消息发送到 Bus 0，Panda 会根据 fwd_hook 配置转发到正确的物理总线。
  """
  addr, dat, bus = packer.make_can_msg(msg_name, 0, values)
  if addr:
    values[checksum_field] = _crc8(dat[:-1])
    return packer.make_can_msg(msg_name, 0, values)
  return addr, dat, bus


def create_can_steer_command(packer, steer, steer_req, cam_msg: dict, counter: int):
  """创建ADAS_LKAS转向控制消息。

  核心逻辑：
  - 不控车时：完整转发原车摄像头数据（保留原车COUNTER）
  - 控车时：只修改转向字段，其他保持原车值

  参数:
    packer: CANPacker实例
    steer: 转向扭矩命令（原始单位，-598到598）
    steer_req: 是否正在主动请求转向（布尔值）- 是否正在控车
    cam_msg: 包含所有字段的原始摄像头消息字典
    counter: 消息计数器（0-15），从原车摄像头消息获取并递增

  返回:
    CAN message tuple (address, data, bus)
  """
  # ADAS_LKAS 默认值 - 当摄像头消息未收到时使用
  # 这些值模拟原车摄像头的"空闲"状态，不会触发车机报错
  ADAS_LKAS_DEFAULTS = {
    "HAND_ON_WHEEL_WARNING": 0,
    "WHEEL_WARNING_CHIME": 0,
    "LKAS_LINE_ACTIVE": 0,
    "LKAS_ENGAGED1": 0,
    "LKA_ENABLE": 1,  # 系统可用
    "LKS_STATUS": 0,
    "STOCK_LKS_AUX": 0,
    "LKS_WARNING_AUDIO_TYPE": 0,
    "LKS_WARNING_TACTILE_TYPE": 0,
    "LKS_ASSIST_MODE": 0,
    "STEER_CMD": 0,
    "STEER_DIR": 0,
    "LDW_READY": 1,  # LDW就绪
    "SET_ME_1": 1,
    "LDW_STEERING": 0,
    "COUNTER": 0,
  }

  # 复制原车摄像头的核心字段，如果字段不存在则使用默认值
  values = _copy_cam_msg_with_defaults(cam_msg, ADAS_LKAS_DEFAULTS)

  # 使用传入的counter（由carcontroller管理递增）
  values["COUNTER"] = counter & 0xF

  # 只在控车时修改转向控制字段
  if steer_req:
    values.update({
      "STEER_CMD": abs(steer),
      "STEER_DIR": steer <= 0,
      "LKAS_ENGAGED1": 1,
      "LKAS_LINE_ACTIVE": 1,
    })
  # 不控车时：完全使用原车摄像头的字段值，不修改任何字段
  # 方向盘警告字段(HAND_ON_WHEEL_WARNING/WHEEL_WARNING_CHIME)保持原车值

  # 重新计算 CHECKSUM（COUNTER变化后需要重新计算）
  return _make_can_msg_with_checksum(packer, "ADAS_LKAS", values)


def create_acc_command(packer, accel, own_fields, cam_msg: dict, counter: int, standstill: bool, is_override: bool = False):
  """创建ACC_CMD纵向控制消息。

  核心逻辑：
  - own_fields=True(真控车 或 踩油门override中): 发OP自己合成的字段(不是原车摄像头的实时值)。
    调用方(carcontroller.py)在gas_override时已经把accel强制为0, 这里CMD/OFFSET系列自然
    算出0物理值、BRAKE_ENGAGED恒为0, 等价于"发自己的ACC_CMD, 加速度0, 不刹车"——不透传原车
    摄像头当时的实时值(★2026-08-26改: 之前override时直接透传cam_msg, 但摄像头那一刻的CMD
    是它自己独立算的瞬时值, 踩油门那一刻如果它正好在算减速, 透传就是一边给油门一边给车发
    刹车指令, 疑似这个冲突触发过原车故障。参考同仓库toyota/honda/gm/vw/hyundai_canfd, 没有
    一家在override时透传外部单元的实时值, 全部是"永远发自己的消息+override时accel归零",
    改成一致做法.)
  - own_fields=False(ACC本身没开): 原样透传原车摄像头 ACC_CMD(仅换COUNTER)，维持消息连续性
    ——这个场景OP完全没有控制意图，两路数据不会冲突。

  参数:
    packer: CANPacker实例
    accel: 期望加速度（m/s²）——gas_override时调用方已强制为0
    own_fields: 是否发OP自己合成的字段（真控车 or 踩油门override中）；False=透传原车
    cam_msg: 包含所有字段的原始摄像头消息字典
    counter: 消息计数器（0-15），由carcontroller管理递增
    standstill: 车辆静止标志

  返回:
    CAN消息元组（地址，数据，总线）
  """
  # ACC_CMD 默认值 - 当摄像头消息未收到时使用
  # 基于CAN日志分析：非激活时raw=140对应的物理值
  # DBC offset: CMD=-138, CMD_OFFSET1=-142, CMD_OFFSET2=-140
  # 物理值 = raw + offset
  ACC_CMD_DEFAULTS = {
    "ACC_REQ": 0,              # ACC未请求
    "NOT_ACC_REQ": 1,          # ACC未请求的反逻辑
    "CRUISE_ENABLE": 1,        # ACC系统可用
    "SET_ME_1": 1,             # 固定值
    "MOTION_CONTROL": 4,       # 非激活时原车用4
    "STANDSTILL2": 0,          # 非静止
    "STATIONARY": 0,           # 原车始终为0
    "CMD": 2,                  # 物理值2 -> raw=140
    "CMD_OFFSET1": -2,         # 物理值-2 -> raw=140
    "CMD_OFFSET2": 0,          # 物理值0 -> raw=140
    "SET_ME_X6A": 0x6A,        # 106，非激活时的值
    "RISING_ENGAGE": 0,        # 非激活
    "BRAKE_ENGAGED": 1,        # 原车始终为1
    "UNKNOWN1": 0,             # 原车始终为0
    "COUNTER": 0,              # 消息计数器（由carcontroller管理递增）
  }

  # 复制原车摄像头的核心字段，如果字段不存在则使用默认值
  values = _copy_cam_msg_with_defaults(cam_msg, ACC_CMD_DEFAULTS)

  # 使用传入的counter（由carcontroller管理递增）
  values["COUNTER"] = counter & 0xF

  # 真控车 或 踩油门override中：发OP自己合成的字段(override时accel已被调用方强制为0)
  if own_fields:
    # 加速度命令（DBC定义offset不同，需要调整物理值使raw值一致）
    #   CMD:         offset=-138
    #   CMD_OFFSET1: offset=-142 (比CMD小4)
    #   CMD_OFFSET2: offset=-140 (比CMD小2)
    accel_phys = accel * 10

    values.update({
      # 加速度命令
      "CMD": accel_phys,
      "CMD_OFFSET1": accel_phys - 4,
      "CMD_OFFSET2": accel_phys - 2,

      # ACC请求标志
      # 2026-08-28修复: 实测原厂14次真实"踩油门"瞬间(含ACC主开关已打开的情况),
      # ACC_REQ 无一例外全是0——原厂设计是踩油门就把请求本身撤回(不只是把加速度调成0),
      # 这样松开油门后ACC_REQ回1、车自己恢复到设定车速。之前这里不管override与否
      # 都无条件发1, 原车ACC会当成"主动请求维持当前车速"跟驾驶员的油门对着干,
      # 这正是用户反馈"踩油门有阻力"的根因。现在override时(accel已被上游强制为0)
      # 跟原厂一样把ACC_REQ也撤回。
      "ACC_REQ": 0 if is_override else 1,
      "NOT_ACC_REQ": 1 if is_override else 0,
      "CRUISE_ENABLE": 1,

      # 运动控制模式: 5=保持静止, 3=加速, 4=刹车, 1=维持速度
      # override时跟原厂非激活态一致用4(配合ACC_REQ=0, 原车根本不会理会这个字段)
      "MOTION_CONTROL": 4 if is_override else (3 if accel > 0 else 4 if accel < 0 else 1),

      "STANDSTILL2": 1 if standstill else 0,
      "RISING_ENGAGE": 0,

      # 固定值
      "SET_ME_X6A": 0x6A,
      "BRAKE_ENGAGED": 0,
      "SET_ME_1": 1,
    })
  # own_fields=False(ACC本身没开): 原样透传原车摄像头ACC_CMD(仅换COUNTER), 维持消息连续性

  # 重新计算 CHECKSUM（COUNTER变化后需要重新计算）
  return _make_can_msg_with_checksum(packer, "ACC_CMD", values)


# 注释掉 create_pcm_buttons - 按用户要求禁用车机定速同步
# def create_pcm_buttons(packer, long_active, set_speed_kmh, counter: int):
#   """创建PCM_BUTTONS消息用于更新车机显示的巡航速度。
#
#   只在控车时发送，告诉车机当前设定速度。
#
#   参数:
#     packer: CANPacker实例
#     long_active: 纵向控制是否激活
#     set_speed_kmh: 设定速度 (km/h)
#     counter: 消息计数器（0-15）
#
#   返回:
#     CAN消息元组（地址，数据，总线）
#   """
#   values = {
#     "ACC_SET_SPEED": set_speed_kmh if long_active else 0,
#     "SET_DISTANCE": 1 if long_active else 0,  # 跟车距离等级
#     "NEW_SIGNAL_1": 3,  # 固定值
#     "ACC_SET": 1 if long_active else 0,  # ACC设置标志
#     "COUNTER": counter & 0xF,
#     "ACC_ON_OFF_BUTTON": 1,  # ACC开关按钮
#   }
#
#   # 重新计算 CHECKSUM
#   return _make_can_msg_with_checksum(packer, "PCM_BUTTONS", values)


