/**
 * @file safety_geely.h
 * @brief 吉利安全钩子
 */

#pragma once

#include "opendbc/safety/declarations.h"

// ========== 常量 ==========

const int GEELY_MAX_STEER = 598;              // 最大转向扭矩 (DBC STEER_CMD 范围)
const int GEELY_MAX_RT_DELTA = 112;           // 实时窗口最大扭矩变化
const int GEELY_MAX_RATE_UP = 20;             // 每帧最大扭矩增加量 (@50Hz)
const int GEELY_MAX_RATE_DOWN = 30;           // 每帧最大扭矩减少量 (@50Hz)
const int GEELY_DRIVER_TORQUE_ALLOWANCE = 124; // 驾驶员扭矩补偿（对齐carstate.py阈值）
const int GEELY_DRIVER_TORQUE_FACTOR = 1;      // 驾驶员扭矩乘数（MAIN_TORQUE已是实际值）

const int GEELY_MAX_ACCEL = 150;
const int GEELY_MIN_ACCEL = -350;
const int GEELY_INACTIVE_ACCEL = 0;

// ========== 配置 ==========

static bool geely_longitudinal = false;
// MAIN_TORQUE is an unsigned magnitude. Its direction is not carried in
// 0x150; use the same angle-delta convention as carstate.py so the Panda and
// host enforce against the same signed driver-torque value.
static int geely_prev_angle = 0;
static int geely_steer_dir = 1;

static const TorqueSteeringLimits GEELY_STEERING_LIMITS = {
  .max_torque = GEELY_MAX_STEER,           // xiaomi8: upstream renamed max_steer -> max_torque
  .max_rate_up = GEELY_MAX_RATE_UP,
  .max_rate_down = GEELY_MAX_RATE_DOWN,
  .max_rt_delta = GEELY_MAX_RT_DELTA,
  .type = TorqueDriverLimited,
  .driver_torque_allowance = GEELY_DRIVER_TORQUE_ALLOWANCE,
  .driver_torque_multiplier = GEELY_DRIVER_TORQUE_FACTOR,
};

static const LongitudinalLimits GEELY_LONG_LIMITS = {
  .max_accel = GEELY_MAX_ACCEL,
  .min_accel = GEELY_MIN_ACCEL,
  .inactive_accel = GEELY_INACTIVE_ACCEL,
};

// ========== RxCheck ==========

#define GEELY_RX_CHECKS \
  {.msg = {{0x0E0, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = false, .frequency = 100U}, { 0 }, { 0 }}},  /* STEERING_MODULE */ \
  {.msg = {{0x150, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = false, .frequency = 50U}, { 0 }, { 0 }}},  /* STEERING_TORQUE */ \
  {.msg = {{0x84,  0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = false, .frequency = 50U}, { 0 }, { 0 }}},  /* GAS_PEDAL */ \
  {.msg = {{0x122, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = false, .frequency = 50U}, { 0 }, { 0 }}},  /* WHEEL_SPEED */ \
  {.msg = {{0x125, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = false, .frequency = 50U}, { 0 }, { 0 }}},  /* PARKING_BRAKE */ \
  {.msg = {{0x1A1, 2, 8, .ignore_checksum = false, .ignore_counter = false, .max_counter = 15U, .ignore_quality_flag = false, .frequency = 50U}, { 0 }, { 0 }}},  /* ACC_CMD Bus 2 (启用校验和和计数器检查) */ \

// ========== 钩子函数 ==========

/**
 * RX钩子 - 处理接收到的CAN消息
 */
static void geely_rx_hook(const CANPacket_t *to_push) {
  int bus = to_push->bus;
  int addr = (int)to_push->addr;

  if (bus == 0) {
    // 转向角度测量
    if (addr == 0x0E0) {
      int angle_meas_new = ((int)to_push->data[0] << 8) | to_push->data[1];
      angle_meas_new = to_signed(angle_meas_new, 16);
      geely_steer_dir = (angle_meas_new >= geely_prev_angle) ? 1 : -1;
      geely_prev_angle = angle_meas_new;
      update_sample(&angle_meas, angle_meas_new);
    }

    // 驾驶员扭矩测量（使用MAIN_TORQUE，与carstate.py保持一致）
    // DBC定义: MAIN_TORQUE : 31|10@0+ (1,0) [0|1023]
    // Motorola位序：字节3的8位 + 字节4的高2位。
    // MAIN_TORQUE包含驾驶员+EPS总扭矩，范围0-1023
    if (addr == 0x150) {
      int torque_magnitude = ((int)to_push->data[3] << 2) | (to_push->data[4] >> 6);
      int torque_driver_new = torque_magnitude * geely_steer_dir;
      update_sample(&torque_driver, torque_driver_new);
    }

    // 油门踏板
    // DBC定义: APPS_1 scale=0.004, 阈值0.01对应原始值 0.01/0.004=2.5，取2
    if (addr == 0x84) {
      gas_pressed = to_push->data[2] > 2U;  // 阈值2 * 0.004 = 0.008，与SP保持一致
    }

    // 刹车踏板
    if (addr == 0x125) {
      brake_pressed = GET_BIT(to_push, 1U);
    }

    // 车速
    if (addr == 0x122) {
      uint16_t speed_raw = ((int)to_push->data[0] << 8) | to_push->data[1];
      int speed_kmh = (speed_raw * 7) / 1000;
      int speed_ms = (speed_kmh * 1000) / 3600;
      UPDATE_VEHICLE_SPEED(speed_ms);
    }
  }

  if (bus == 2) {
    // ACC_CMD消息 - 巡航控制状态处理
    if (addr == 0x1A1) {
      bool cruise_enable = GET_BIT(to_push, 37U);
      bool not_acc_req = GET_BIT(to_push, 12U);
      bool acc_req = GET_BIT(to_push, 36U);

      // ACC主开关状态
      acc_main_on = cruise_enable;

      // ACC激活逻辑：同时满足acc_req且非not_acc_req
      bool cruise_engaged = acc_req && !not_acc_req;
      pcm_cruise_check(cruise_engaged);

      // controls_allowed 基于原厂ACC激活状态
      if (cruise_engaged) {
        controls_allowed = true;
      } else if (!cruise_enable) {
        // ACC主开关关闭时禁用所有控制
        controls_allowed = false;
      }
      // cruise_engaged下降沿由pcm_cruise_check处理
    }
  }
}
/**
 * TX钩子 - 验证发送消息
 */
static bool geely_tx_hook(const CANPacket_t *to_send) {
  bool tx = true;
  int addr = (int)to_send->addr;

  // ADAS_LKAS转向命令检查
  if (addr == 0x1B0) {
    // DBC STEER_CMD is 47|11@0: byte5 followed by byte6[7:5].
    // Do not use byte4: doing so under-reports the sent torque to safety.
    uint8_t b5 = to_send->data[5];
    uint8_t b6 = to_send->data[6];
    int steer_abs = ((int)b5 << 3) | (b6 >> 5);

    bool steer_left = (b6 & 0x10) != 0;

    int desired_torque = steer_left ? -steer_abs : steer_abs;
    int steer_req = (to_send->data[1] & 0x10) != 0 ? 1 : 0;

    // 转向扭矩检查
    if (steer_torque_cmd_checks(desired_torque, steer_req, GEELY_STEERING_LIMITS)) {
      tx = false;
    }

    // 注意:不在tx_hook检查controls_allowed!
    // controls_allowed只在rx_hook中使用,消息发送时机由上层carcontroller控制
    // 参考Toyota/Honda/GM的实现
  }

  // ACC_CMD纵向命令检查（参考SP版本）
  if (addr == 0x1A1 && geely_longitudinal) {
    int accel_raw = to_send->data[3];
    int accel_phys = accel_raw - 138;
    int accel_cmd = accel_phys * 10;

    // 区分控车/不控车状态的检查逻辑
    if (controls_allowed) {
      // 激活控制：严格的加速度范围检查 [-350, 150]
      if ((accel_cmd > GEELY_LONG_LIMITS.max_accel) || (accel_cmd < GEELY_LONG_LIMITS.min_accel)) {
        tx = false;
      }
    } else {
      // 非激活控制：基本范围检查（防止明显错误数据）
      if ((accel_raw < 50) || (accel_raw > 230)) {
        tx = false;
      }
    }
  }

  // LKAS HUD曲率检查
  if (addr == 0x1B2) {
    uint8_t b3 = to_send->data[3];
    int curvature_raw = b3 & 0x7F;
    int curvature = curvature_raw - 64;

    if ((curvature < -63) || (curvature > 63)) {
      tx = false;
    }
  }

  return tx;
}

/**
 * FWD钩子 - 消息转发控制
 *
 * 返回值语义（参考Toyota/Honda）：
 * - -1: 不转发（阻止消息）
 * -  0: 转发到bus 0
 * -  2: 转发到bus 2
 *
 * 转发逻辑：
 * - bus 0 → bus 2（车辆到摄像头）
 * - bus 2 → bus 0（摄像头到车辆），但阻止LKAS和ACC消息
 */
// xiaomi8: upstream changed signature to `bool fwd_hook(int bus, int addr)`
// returns true if message should be BLOCKED from forwarding (framework does default fwd).
static bool geely_fwd_hook(int bus_num, int addr) {
  // bus 2 (摄像头) → bus 0 (车辆): block LKAS always, block ACC when long
  if (bus_num == 2) {
    if (addr == 0x1B0) return true;                  // LKAS 始终拦截
    if (geely_longitudinal && addr == 0x1A1) return true;  // ACC 仅在 long 控制时拦截
  }
  return false;
}

/**
 * Init函数 - 配置安全模式
 */
static safety_config geely_init(uint16_t param) {
  const uint16_t GEELY_PARAM_LONG = 1;
  geely_longitudinal = GET_FLAG(param, GEELY_PARAM_LONG);

  static RxCheck geely_rx_checks[] = {
    GEELY_RX_CHECKS
  };

  static const CanMsg GEELY_TX_MSGS[] = {
    {0x1B0, 0, 8, .check_relay = true},   // ADAS_LKAS (发送到Bus 0，Panda转发)
  };

  static const CanMsg GEELY_LONG_TX_MSGS[] = {
    {0x1B0, 0, 8, .check_relay = true},   // ADAS_LKAS
    {0x1A1, 0, 8, .check_relay = true},   // ACC_CMD
  };

  safety_config ret;
  if (geely_longitudinal) {
    ret = BUILD_SAFETY_CFG(geely_rx_checks, GEELY_LONG_TX_MSGS);
  } else {
    ret = BUILD_SAFETY_CFG(geely_rx_checks, GEELY_TX_MSGS);
  }

  return ret;
}

// ========== 校验和和计数器函数 ==========

static uint32_t geely_get_checksum(const CANPacket_t *to_push) {
  int addr = to_push->addr;
  int len = GET_LEN(to_push);

  // 根据消息类型确定校验和位置
  if ((addr == 0x200) || (addr == 0x202)) {
    // GAS_COMMAND 和 TORQUE_COMMAND: checksum 在字节7
    return (uint8_t)to_push->data[7];
  } else if (addr == 0x1A1) {
    // ACC_CMD: checksum 在字节7
    return (uint8_t)to_push->data[7];
  } else {
    // 其他消息默认在最后一个字节
    return (uint8_t)to_push->data[len - 1];
  }
}

static uint32_t geely_compute_checksum(const CANPacket_t *to_push) {
  int len = GET_LEN(to_push);
  uint8_t crc = 0xFF;  // 初始化值

  // CRC8 计算：多项式 0x2F，输出异或 0xFF
  for (int i = 0; i < len - 1; i++) {  // 排除最后一个字节（校验和字节）
    uint8_t byte = to_push->data[i];
    crc ^= byte;
    for (int j = 0; j < 8; j++) {
      if (crc & 0x80) {
        crc = (crc << 1) ^ 0x2F;
      } else {
        crc <<= 1;
      }
    }
  }

  return crc ^ 0xFF;  // 输出异或
}

static uint8_t geely_get_counter(const CANPacket_t *to_push) {
  int addr = to_push->addr;

  // 根据消息类型确定计数器位置
  if ((addr == 0x200) || (addr == 0x202)) {
    // GAS_COMMAND 和 TORQUE_COMMAND: counter 在字节6的位3-6 (51|4@0+)
    // 起始位51 = 字节6*8+3，长度4位
    return (to_push->data[6] >> 3) & 0xF;
  } else if (addr == 0x1A1) {
    // ACC_CMD: counter 在字节5的位0-3 (40|4@1+)
    return to_push->data[5] & 0xF;
  } else {
    // 其他消息没有计数器，返回0
    return 0;
  }
}

// ========== 钩子结构 ==========

const safety_hooks geely_hooks = {
  .init = geely_init,
  .rx = geely_rx_hook,
  .tx = geely_tx_hook,
  .fwd = geely_fwd_hook,
  .get_checksum = geely_get_checksum,
  .compute_checksum = geely_compute_checksum,
  .get_counter = geely_get_counter,
  .get_quality_flag_valid = 0,
};
