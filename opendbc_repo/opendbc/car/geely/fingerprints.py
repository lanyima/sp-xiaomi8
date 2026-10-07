"""吉利车辆指纹用于识别。"""
from opendbc.car.geely.values import CAR
from opendbc.car.structs import CarParams

Ecu = CarParams.Ecu

Fingerprints = {
  CAR.GEELY_BINYUE: [{
    # PT总线(Bus 0) - 基本车辆状态消息
    130: 8,   # ENGINE (0x82)
    132: 8,   # GAS_PEDAL (0x84)
    133: 8,   # 0x85
    151: 8,   # 0x97
    224: 8,   # STEERING_MODULE (0x0E0)
    275: 8,   # TRANSMISSION (0x113)
    288: 8,   # 0x120
    289: 8,   # 0x121
    290: 8,   # WHEEL_SPEED (0x122)
    291: 8,   # BRAKE (0x123)
    292: 8,   # 0x124
    293: 8,   # PARKING_BRAKE (0x125)
    294: 8,   # FCW (0x126)
    295: 8,   # 0x127
    298: 8,   # 0x12A
    304: 8,   # 0x130
    305: 8,   # 0x131
    336: 8,   # STEERING_TORQUE (0x150)
    400: 8,   # 0x190
    401: 8,   # BSM_ADAS (0x191)
    417: 8,   # ACC_CMD (0x1A1)
    418: 8,   # 0x1A2
    419: 8,   # PCM_BUTTONS (0x1A3)
    422: 8,   # ADAS_LEAD_DETECT (0x1A6)
    423: 8,   # ADAS_CAR_DETECT (0x1A7)
    432: 8,   # ADAS_LKAS (0x1B0)
    434: 8,   # LKAS HUD (0x1B2) - 原车摄像头发送，用于指纹识别
    482: 8,   # 0x1E2
    496: 8,   # 0x1F0
    536: 8,   # 0x218
    608: 8,   # 0x260
    621: 8,   # 0x26D
    641: 8,   # LEFT_STALK (0x281)
    643: 8,   # ACC_BUTTONS (0x283)
    644: 8,   # 0x284
    645: 8,   # DOOR_LEFT_SIDE (0x285)
    646: 8,   # DOOR_RIGHT_SIDE (0x286)
    650: 8,   # 0x28A
    652: 8,   # 0x28C
    654: 8,   # 0x28E
    658: 8,   # 0x292
    672: 8,   # 0x2A0
    676: 8,   # POWER_MODE (0x2A4)
    679: 8,   # 0x2A7
    680: 8,   # 0x2A8
    682: 8,   # 0x2AA
    686: 8,   # 0x2AE
    692: 8,   # 0x2B4
    736: 8,   # 0x2E0
    753: 8,   # AIRCON (0x2F1)
    784: 8,   # 0x310
    896: 8,   # SEATBELTS (0x380)
    912: 8,   # 0x390
    993: 8,   # 0x3E1
    994: 8,   # 0x3E2
    1008: 8,  # 0x3F0
    1009: 8,  # 0x3F1
    1033: 8,  # 0x409
    1036: 8,  # 0x40C
    1039: 8,  # 0x40F
    1058: 8,  # 0x422
  }],
}

# 固件版本字典
# 吉利车辆通常不通过UDS公开固件版本
# 这是一个最小结构以满足框架要求
FW_VERSIONS = {
  CAR.GEELY_BINYUE: {
    (Ecu.eps, 0x730, None): [
      b'GEELY_EPS_V1.0',
    ],
    (Ecu.engine, 0x7e0, None): [
      b'GEELY_ECU_V1.0',
    ],
    (Ecu.fwdCamera, 0x750, None): [
      b'GEELY_CAM_V1.0',
    ],
  },
}

# Export fingerprints using enum keys directly (not string keys)
# The framework will handle the conversion automatically
FINGERPRINTS = Fingerprints
