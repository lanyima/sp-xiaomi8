#!/usr/bin/env python3
"""
sensord_imu_qmi.py - 小米8 IMU sensord (QMI → SLPI → ICM-20690 → cereal)

替代 openpilot 原始 sensord.py, 通过 QMI 从 SLPI 读取 ICM-20690 
加速度计+陀螺仪数据, 发布 cereal accelerometer/gyroscope 消息.

架构:
  SLPI DSP ← SPI → ICM-20690
      ↓ QMI (AF_MSM_IPC)
  This script
      ↓ cereal PubMaster
  locationd / openpilot

用法:
  sudo python3 sensord_imu_qmi.py          # 正常运行
  sudo python3 sensord_imu_qmi.py --test   # 测试模式(打印数据不发 cereal)
  sudo python3 sensord_imu_qmi.py --rate 100  # 指定采样率

已验证: ICM-20690 通过 SLPI QMI 返回实时加速度计/陀螺仪数据 (100Hz)
"""

import socket
import struct
import ctypes
import ctypes.util
import select
import sys
import time
import os
import errno
import signal
import math
import threading

# ======== 常量 ========
AF_MSM_IPC = 27
MSM_IPC_ADDR_ID = 2
IPC_ROUTER_IOCTL_MAGIC = 0xC3

SUID_LOOKUP_LOW  = 0xabababababababab
SUID_LOOKUP_HIGH = 0xabababababababab

SNS_SUID_MSGID_SNS_SUID_REQ   = 512
SNS_STD_SENSOR_MSGID_CONFIG    = 513
SNS_STD_SENSOR_MSGID_EVENT     = 1025

QMI_SNS_CLIENT_SVC_ID = 400
QMI_MSG_ID_REPORT      = 0x0020

# ICM-20690 SUIDs (discovered via SUID lookup)
ACCEL_SUID_LOW  = 0x58583630324d4349  # "ICM206XX"
ACCEL_SUID_HIGH = 0x305f4c454343415f  # "_ACCEL_0"
GYRO_SUID_LOW   = 0x58583630324d4349  # "ICM206XX"
GYRO_SUID_HIGH  = 0x30305f4f5259475f  # "_GYRO_00"

# openpilot sensor types
SENSOR_ACCELEROMETER = 1
SENSOR_GYRO_UNCALIBRATED = 5


# ======== Protobuf 编码 ========

def pb_varint_encode(val):
    buf = bytearray()
    while val > 0x7F:
        buf.append((val & 0x7F) | 0x80)
        val >>= 7
    buf.append(val & 0x7F)
    return bytes(buf)

def pb_tag(field_num, wire_type):
    return pb_varint_encode((field_num << 3) | wire_type)

def pb_field_varint(field_num, val):
    return pb_tag(field_num, 0) + pb_varint_encode(val)

def pb_field_fixed64(field_num, val):
    return pb_tag(field_num, 1) + struct.pack('<Q', val)

def pb_field_bytes(field_num, data):
    return pb_tag(field_num, 2) + pb_varint_encode(len(data)) + data

def pb_field_string(field_num, s):
    return pb_field_bytes(field_num, s.encode('utf-8'))

def pb_field_fixed32(field_num, val):
    return pb_tag(field_num, 5) + struct.pack('<I', val)

def pb_varint_decode(data, pos):
    val = 0
    shift = 0
    while pos < len(data):
        b = data[pos]
        val |= (b & 0x7F) << shift
        shift += 7
        pos += 1
        if not (b & 0x80):
            break
    return val, pos

def pb_fields(data):
    """解析 protobuf 字段"""
    fields = []
    pos = 0
    while pos < len(data):
        if pos >= len(data):
            break
        tag, pos = pb_varint_decode(data, pos)
        field_num = tag >> 3
        wire_type = tag & 7
        
        if wire_type == 0:
            val, pos = pb_varint_decode(data, pos)
            fields.append((field_num, 0, val))
        elif wire_type == 1:
            if pos + 8 > len(data): break
            val = struct.unpack('<Q', data[pos:pos+8])[0]
            pos += 8
            fields.append((field_num, 1, val))
        elif wire_type == 2:
            slen, pos = pb_varint_decode(data, pos)
            if pos + slen > len(data): break
            val = data[pos:pos+slen]
            pos += slen
            fields.append((field_num, 2, val))
        elif wire_type == 5:
            if pos + 4 > len(data): break
            val = struct.unpack('<I', data[pos:pos+4])[0]
            pos += 4
            fields.append((field_num, 5, val))
        else:
            break
    return fields


# ======== MSM IPC ========

class msm_ipc_port_addr(ctypes.Structure):
    _fields_ = [("node_id", ctypes.c_uint32), ("port_id", ctypes.c_uint32)]

class msm_ipc_port_name(ctypes.Structure):
    _fields_ = [("service", ctypes.c_uint32), ("instance", ctypes.c_uint32)]

class msm_ipc_addr_union(ctypes.Union):
    _fields_ = [("port_addr", msm_ipc_port_addr), ("port_name", msm_ipc_port_name)]

class msm_ipc_addr(ctypes.Structure):
    _fields_ = [("addrtype", ctypes.c_uint8), ("addr", msm_ipc_addr_union)]

class sockaddr_msm_ipc(ctypes.Structure):
    _fields_ = [
        ("family", ctypes.c_uint16),
        ("address", msm_ipc_addr),
        ("reserved", ctypes.c_uint8),
    ]

class msm_ipc_server_info(ctypes.Structure):
    _fields_ = [
        ("node_id", ctypes.c_uint32), ("port_id", ctypes.c_uint32),
        ("service", ctypes.c_uint32), ("instance", ctypes.c_uint32),
    ]

class server_lookup_args(ctypes.Structure):
    _fields_ = [
        ("port_name", msm_ipc_port_name),
        ("num_entries_in_array", ctypes.c_int),
        ("num_entries_found", ctypes.c_int),
        ("lookup_mask", ctypes.c_uint32),
        ("srv_info", msm_ipc_server_info * 8),
    ]

import fcntl

libc = ctypes.CDLL("libc.so.6", use_errno=True)


class SNSQMIClient:
    """SLPI 传感器 QMI 客户端"""
    
    def __init__(self):
        self.sock = None
        self.node = 0
        self.port = 0
        self.txn_id = 0
    
    def connect(self, max_retries=15, retry_delay=2.0):
        """连接到 SNS_CLIENT_SVC, 等待 SLPI DSP 就绪 (最多重试 max_retries 次)"""
        last_err = None
        for attempt in range(max_retries):
            try:
                if self.sock is not None:
                    try:
                        self.sock.close()
                    except Exception:
                        pass
                self.sock = socket.socket(AF_MSM_IPC, socket.SOCK_DGRAM, 0)

                args = server_lookup_args()
                args.port_name.service = QMI_SNS_CLIENT_SVC_ID
                args.port_name.instance = 0
                args.num_entries_in_array = 8
                args.lookup_mask = 0

                size = ctypes.sizeof(sockaddr_msm_ipc)
                ioctl_num = (3 << 30) | (IPC_ROUTER_IOCTL_MAGIC << 8) | 2 | (size << 16)
                fcntl.ioctl(self.sock.fileno(), ioctl_num, args)

                if args.num_entries_found == 0:
                    raise RuntimeError("SNS_CLIENT_SVC not found (SLPI not ready)")

                self.node = args.srv_info[0].node_id
                self.port = args.srv_info[0].port_id
                print(f"[SNS] 已连接: node={self.node} port={self.port}")
                return  # 成功
            except (RuntimeError, OSError) as e:
                last_err = e
                if attempt < max_retries - 1:
                    print(f"[SNS] 连接失败 ({attempt+1}/{max_retries}): {e}, {retry_delay:.0f}s 后重试...")
                    time.sleep(retry_delay)
        raise RuntimeError(f"SNS_CLIENT_SVC 连接失败 (已重试 {max_retries} 次): {last_err}")
    
    def _next_txn(self):
        self.txn_id = (self.txn_id + 1) & 0xFFFF
        return self.txn_id
    
    def _send(self, pb_payload):
        """发送 QMI 消息"""
        txn = self._next_txn()
        
        # QMI header
        qmi = bytearray()
        qmi.append(0x00)  # request
        qmi += struct.pack('<H', txn)
        qmi += struct.pack('<H', QMI_MSG_ID_REPORT)
        
        # TLV 0x01
        tlv_data = struct.pack('<H', len(pb_payload)) + pb_payload
        qmi += struct.pack('<H', 1 + 2 + len(tlv_data))
        qmi.append(0x01)
        qmi += struct.pack('<H', len(tlv_data))
        qmi += tlv_data
        
        addr = sockaddr_msm_ipc()
        addr.family = AF_MSM_IPC
        addr.address.addrtype = MSM_IPC_ADDR_ID
        addr.address.addr.port_addr.node_id = self.node
        addr.address.addr.port_addr.port_id = self.port
        
        buf = ctypes.create_string_buffer(bytes(qmi))
        ret = libc.sendto(self.sock.fileno(), buf, len(qmi), 0,
                          ctypes.byref(addr), ctypes.sizeof(addr))
        if ret < 0:
            raise OSError(f"sendto failed: {os.strerror(ctypes.get_errno())}")
        return txn
    
    def _recv(self, timeout_ms=100):
        """接收 QMI 消息, 返回 (type, msg_id, tlvs) 或 None"""
        ready = select.select([self.sock], [], [], timeout_ms / 1000.0)
        if not ready[0]:
            return None
        
        rxbuf = ctypes.create_string_buffer(65536)
        from_addr = sockaddr_msm_ipc()
        fromlen = ctypes.c_int(ctypes.sizeof(from_addr))
        
        n = libc.recvfrom(self.sock.fileno(), rxbuf, 65536, 0,
                          ctypes.byref(from_addr), ctypes.byref(fromlen))
        if n <= 0:
            return None
        
        data = rxbuf.raw[:n]
        if n < 7:
            return None
        
        msg_type = data[0]
        msg_id = struct.unpack('<H', data[3:5])[0]
        
        # Parse TLVs
        tlvs = {}
        pos = 7
        while pos + 3 <= n:
            t = data[pos]
            l = struct.unpack('<H', data[pos+1:pos+3])[0]
            if pos + 3 + l > n: break
            tlvs[t] = data[pos+3:pos+3+l]
            pos += 3 + l
        
        return (msg_type, msg_id, tlvs)
    
    def _build_request(self, suid_low, suid_high, msg_id, payload_bytes=b''):
        """构建 sns_client_request_msg"""
        suid = pb_field_fixed64(1, suid_low) + pb_field_fixed64(2, suid_high)
        susp = pb_field_varint(1, 1) + pb_field_varint(2, 0)  # APSS, WAKEUP
        request = pb_field_bytes(2, payload_bytes) if payload_bytes else b''
        
        msg = pb_field_bytes(1, suid)
        msg += pb_field_fixed32(2, msg_id)
        msg += pb_field_bytes(3, susp)
        msg += pb_field_bytes(4, request)
        return msg
    
    def lookup_suid(self, data_type):
        """查询传感器 SUID"""
        suid_req = pb_field_string(1, data_type)
        pb = self._build_request(SUID_LOOKUP_LOW, SUID_LOOKUP_HIGH,
                                  SNS_SUID_MSGID_SNS_SUID_REQ, suid_req)
        self._send(pb)
        
        # 等待 RESP + IND
        deadline = time.time() + 5
        while time.time() < deadline:
            result = self._recv(500)
            if result is None:
                continue
            msg_type, msg_id, tlvs = result
            if msg_type == 4 and 0x02 in tlvs:  # IND
                payload = tlvs[0x02]
                if len(payload) >= 2:
                    pb_len = struct.unpack('<H', payload[0:2])[0]
                    pb_data = payload[2:2+pb_len]
                    return self._extract_suid(pb_data, data_type)
        return None, None
    
    def _extract_suid(self, pb_data, data_type):
        """从 sns_suid_event 提取 SUID"""
        for f_num, wire, val in pb_fields(pb_data):
            if f_num == 2 and wire == 2:
                for ef_num, ef_wire, ef_val in pb_fields(val):
                    if ef_num == 3 and ef_wire == 2:
                        for sf_num, sf_wire, sf_val in pb_fields(ef_val):
                            if sf_num == 2 and sf_wire == 2:
                                low = high = 0
                                for uid_num, uid_wire, uid_val in pb_fields(sf_val):
                                    if uid_num == 1 and uid_wire == 1: low = uid_val
                                    if uid_num == 2 and uid_wire == 1: high = uid_val
                                return low, high
        return None, None
    
    def start_streaming(self, suid_low, suid_high, sample_rate=100.0):
        """启动传感器数据流"""
        # sns_std_sensor_config { sample_rate = float }
        config = pb_tag(1, 5) + struct.pack('<f', sample_rate)
        
        # batch_period = 0 (不批处理)
        batch = pb_field_varint(1, 0)
        request = pb_field_bytes(1, batch) + pb_field_bytes(2, config)
        
        # Build full request
        suid = pb_field_fixed64(1, suid_low) + pb_field_fixed64(2, suid_high)
        susp = pb_field_varint(1, 1) + pb_field_varint(2, 0)
        
        msg = pb_field_bytes(1, suid)
        msg += pb_field_fixed32(2, SNS_STD_SENSOR_MSGID_CONFIG)
        msg += pb_field_bytes(3, susp)
        msg += pb_field_bytes(4, request)
        
        self._send(msg)
        
        # 等待 RESP
        deadline = time.time() + 3
        while time.time() < deadline:
            result = self._recv(500)
            if result and result[0] == 2:  # RESP
                if 0x02 in result[2]:
                    r = result[2][0x02]
                    if len(r) >= 2 and struct.unpack('<H', r[0:2])[0] == 0:
                        return True
        return False
    
    def recv_sensor_event(self, timeout_ms=50):
        """接收传感器数据事件, 返回 (suid_low, suid_high, msg_id, timestamp, samples, status) 或 None"""
        result = self._recv(timeout_ms)
        if result is None:
            return None
        
        msg_type, qmi_msg_id, tlvs = result
        if msg_type != 4 or 0x02 not in tlvs:  # Not IND
            return None
        
        payload = tlvs[0x02]
        if len(payload) < 2:
            return None
        
        pb_len = struct.unpack('<H', payload[0:2])[0]
        pb_data = payload[2:2+pb_len]
        
        return self._parse_sensor_event(pb_data)
    
    def _parse_sensor_event(self, pb_data):
        """
        解析 sns_client_event_msg → 提取传感器数据
        返回 (suid_low, suid_high, events_list)
        events_list = [(msg_id, timestamp_ticks, [float_samples], status), ...]
        """
        suid_low = suid_high = 0
        events = []
        
        for f_num, wire, val in pb_fields(pb_data):
            if f_num == 1 and wire == 2:  # suid submessage
                for uid_num, uid_wire, uid_val in pb_fields(val):
                    if uid_num == 1 and uid_wire == 1: suid_low = uid_val
                    if uid_num == 2 and uid_wire == 1: suid_high = uid_val
            
            elif f_num == 2 and wire == 2:  # event submessage
                msg_id = 0
                timestamp = 0
                samples = []
                status = 0
                
                for ef_num, ef_wire, ef_val in pb_fields(val):
                    if ef_num == 1 and ef_wire == 5:  # msg_id (fixed32)
                        msg_id = ef_val
                    elif ef_num == 2 and ef_wire == 1:  # timestamp (fixed64)
                        timestamp = ef_val
                    elif ef_num == 3 and ef_wire == 2:  # payload
                        # sns_std_sensor_event { data = repeated float, status = enum }
                        for sf_num, sf_wire, sf_val in pb_fields(ef_val):
                            if sf_num == 1 and sf_wire == 5:  # float sample
                                samples.append(struct.unpack('<f', struct.pack('<I', sf_val))[0])
                            elif sf_num == 2 and sf_wire == 0:  # status
                                status = sf_val
                
                events.append((msg_id, timestamp, samples, status))
        
        return (suid_low, suid_high, events)
    
    def close(self):
        if self.sock:
            self.sock.close()
            self.sock = None


# ======== SLPI 时间戳 → 纳秒 转换 ========

# SLPI 使用 QTimer 时钟 (19.2 MHz)
# QTimer 从 SoC 上电就开始计数 (包括 bootloader)
# CLOCK_MONOTONIC 从 Linux 内核启动后才开始
# 两者的差值 = bootloader 执行时间 (通常 1-5 秒)
# locationd 要求 sensor_timestamp 与 logMonoTime 差值 < 100ms
# 因此需要校准偏移, 将 QTimer 时间戳对齐到 CLOCK_MONOTONIC
QTIMER_FREQ = 19200000

# QTimer→CLOCK_MONOTONIC 校准偏移量 (纳秒)
# 持续用最大值校准: measured_offset = true_offset - qmi_delay
# 最大 measured_offset 对应最小 qmi_delay, 最接近 true_offset
_qtimer_boot_offset = None

def qtimer_to_ns(ticks):
    """QTimer ticks → 纳秒 (原始, 未校准)"""
    return int(ticks * 1_000_000_000 / QTIMER_FREQ)

def qtimer_to_boottime_ns(ticks):
    """QTimer ticks → CLOCK_MONOTONIC 对齐的纳秒

    持续更新校准: 启动时系统高负载导致 QMI 延迟 100ms+,
    单次校准会将该延迟烙入偏移量. 改为取历史最大偏移值,
    对应最小 QMI 延迟, 自动收敛到真实偏移.
    """
    global _qtimer_boot_offset

    qtimer_ns = int(ticks * 1_000_000_000 / QTIMER_FREQ)
    boot_ns = time.monotonic_ns()
    current_offset = qtimer_ns - boot_ns

    if _qtimer_boot_offset is None:
        _qtimer_boot_offset = current_offset
        print(f"[sensord] QTimer 初始偏移: {_qtimer_boot_offset / 1e6:.1f} ms")
    elif current_offset > _qtimer_boot_offset:
        old_ms = _qtimer_boot_offset / 1e6
        _qtimer_boot_offset = current_offset
        print(f"[sensord] QTimer 偏移校正: {old_ms:.1f} → {_qtimer_boot_offset / 1e6:.1f} ms")

    return qtimer_ns - _qtimer_boot_offset


# ======== sensord 主循环 ========

running = True

def signal_handler(sig, frame):
    global running
    running = False
    print("\n[sensord] 收到信号, 停止中...")

def run_sensord(sample_rate=100.0, test_mode=False):
    """运行 sensord, 读取 IMU 数据并发布 cereal 消息"""
    global running
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    print(f"[sensord] 小米8 IMU QMI 桥接 (ICM-20690 via SLPI)")
    print(f"[sensord] 采样率: {sample_rate} Hz, 测试模式: {test_mode}")
    
    # 连接 QMI
    client = SNSQMIClient()
    client.connect()
    
    # 验证 SUID
    print("[sensord] 验证加速度计 SUID...")
    a_low, a_high = client.lookup_suid("accel")
    if a_low is None:
        print("[sensord] ❌ 加速度计 SUID 未找到!")
        return 1
    print(f"[sensord] ✅ accel SUID: 0x{a_low:016x}:0x{a_high:016x}")
    
    # 需要新连接来查询 gyro (每个连接只处理一个请求)
    client.close()
    client = SNSQMIClient()
    client.connect()
    
    print("[sensord] 验证陀螺仪 SUID...")
    g_low, g_high = client.lookup_suid("gyro")
    if g_low is None:
        print("[sensord] ❌ 陀螺仪 SUID 未找到!")
        return 1
    print(f"[sensord] ✅ gyro SUID: 0x{g_low:016x}:0x{g_high:016x}")
    
    # 重新连接, 启动两个传感器流
    client.close()
    client = SNSQMIClient()
    client.connect()
    
    print(f"[sensord] 启动加速度计流 ({sample_rate} Hz)...")
    if not client.start_streaming(a_low, a_high, sample_rate):
        print("[sensord] ❌ 加速度计启动失败!")
        return 1
    print("[sensord] ✅ 加速度计已启动")
    
    # 陀螺仪用同一个连接
    print(f"[sensord] 启动陀螺仪流 ({sample_rate} Hz)...")
    if not client.start_streaming(g_low, g_high, sample_rate):
        print("[sensord] ⚠️ 陀螺仪启动失败, 尝试继续...")
    else:
        print("[sensord] ✅ 陀螺仪已启动")
    
    # cereal 初始化 (只在非测试模式)
    pm = None
    if not test_mode:
        try:
            import cereal.messaging as messaging
            pm = messaging.PubMaster(['accelerometer', 'gyroscope'])
            print("[sensord] ✅ cereal PubMaster 已创建")
        except ImportError:
            print("[sensord] ⚠️ cereal 不可用, 切换到测试模式")
            test_mode = True
    
    # 数据接收循环
    accel_count = 0
    gyro_count = 0
    start_time = time.time()
    last_print = start_time
    
    print("[sensord] 开始接收数据...")
    
    while running:
        result = client.recv_sensor_event(timeout_ms=20)
        if result is None:
            continue
        
        suid_low, suid_high, events = result
        
        for msg_id, timestamp_ticks, samples, status in events:
            if msg_id != SNS_STD_SENSOR_MSGID_EVENT:
                continue
            if len(samples) < 3:
                continue
            
            ts_ns = qtimer_to_boottime_ns(timestamp_ticks)
            x, y, z = samples[0], samples[1], samples[2]
            
            # 判断是加速度计还是陀螺仪
            is_accel = (suid_high == a_high)
            is_gyro = (suid_high == g_high)
            
            if is_accel:
                accel_count += 1
                if not test_mode and pm:
                    try:
                        import cereal.messaging as messaging
                        from cereal import log
                        msg = messaging.new_message('accelerometer', valid=True)
                        msg.accelerometer.version = 1
                        msg.accelerometer.sensor = SENSOR_ACCELEROMETER
                        msg.accelerometer.type = 1
                        msg.accelerometer.timestamp = ts_ns
                        # locationd 内部: meas=[-v[2],-v[1],-v[0]], KF期望静止时 meas[2]=-9.81
                        # v[0]=+9.81(上), v[1]=左, v[2]=前 | SLPI_X(开机键)朝上, -SLPI_Z(朝屏)取反=前
                        acc = msg.accelerometer.init('acceleration')
                        acc.v = [x, y, -z]  # Mi8横屏: v[0]=SLPI_X(上≈+9.81), v[1]=SLPI_Y(左), v[2]=-SLPI_Z(前)
                        acc.status = status
                        msg.accelerometer.source = log.SensorEventData.SensorSource.lsm6ds3
                        pm.send('accelerometer', msg)
                    except Exception as e:
                        if accel_count <= 3:
                            print(f"[sensord] cereal 发送错误: {e}")
                
            elif is_gyro:
                gyro_count += 1
                if not test_mode and pm:
                    try:
                        import cereal.messaging as messaging
                        from cereal import log
                        msg = messaging.new_message('gyroscope', valid=True)
                        msg.gyroscope.version = 1
                        msg.gyroscope.sensor = SENSOR_GYRO_UNCALIBRATED
                        msg.gyroscope.type = 16
                        msg.gyroscope.timestamp = ts_ns
                        gyro = msg.gyroscope.init('gyroUncalibrated')
                        # locationd: v[0]=上轴角速度(yaw), v[1]=左轴(pitch), v[2]=前轴(roll)
                        # ICM-20690 Z轴标准: SLPI_Z=朝屏(后), -SLPI_Z=前, SLPI_X=上(yaw轴)
                        gyro.v = [x, y, -z]  # Mi8横屏: v[0]=SLPI_X(上), v[1]=SLPI_Y(左), v[2]=-SLPI_Z(前)
                        gyro.status = status
                        msg.gyroscope.source = log.SensorEventData.SensorSource.lsm6ds3
                        pm.send('gyroscope', msg)
                    except Exception as e:
                        if gyro_count <= 3:
                            print(f"[sensord] cereal 发送错误: {e}")
            
            # 测试模式打印
            now = time.time()
            if test_mode and now - last_print >= 1.0:
                elapsed = now - start_time
                a_rate = accel_count / elapsed if elapsed > 0 else 0
                g_rate = gyro_count / elapsed if elapsed > 0 else 0
                
                sensor_name = "ACCEL" if is_accel else "GYRO " if is_gyro else "???  "
                print(f"[{elapsed:.1f}s] {sensor_name} x={x:8.4f} y={y:8.4f} z={z:8.4f} "
                      f"| rates: accel={a_rate:.1f}Hz gyro={g_rate:.1f}Hz "
                      f"(total: {accel_count}+{gyro_count})")
                last_print = now
    
    elapsed = time.time() - start_time
    print(f"\n[sensord] 停止. 运行时间: {elapsed:.1f}s")
    print(f"[sensord] 加速度计: {accel_count} 读数 ({accel_count/elapsed:.1f} Hz)")
    print(f"[sensord] 陀螺仪:   {gyro_count} 读数 ({gyro_count/elapsed:.1f} Hz)")
    
    client.close()
    return 0


def main():
    """Entry point for openpilot manager"""
    # sensord_qmi needs root for AF_MSM_IPC (QMI) socket access
    if os.getuid() != 0:
        print('[sensord] Escalating to root for QMI access...')
        os.execvp('sudo', ['sudo', '-E', sys.executable, '-c',
                   "import os, sys; os.chdir('/data/openpilot'); sys.path.insert(0, '.'); "
                   "from system.sensord.sensord_qmi import run_sensord_as_root; run_sensord_as_root()"])
    run_sensord_as_root()


def run_sensord_as_root():
    """Run sensord with root privileges, with retry on SLPI/QMI failure.

    SLPI DSP 启动可能晚于 openpilot manager, 导致 QMI 连接失败.
    此处用指数退避重试, 避免崩溃循环占满 CPU 和 manager 日志.
    """
    import glob

    MAX_RETRIES = 30      # 最多重试 30 次 (覆盖 ~5 分钟启动窗口)
    INITIAL_DELAY = 2.0   # 初始等待 2s
    MAX_DELAY = 15.0      # 最大等待 15s

    delay = INITIAL_DELAY
    for attempt in range(MAX_RETRIES):
        # 每次重试前清理旧 shm
        for f in glob.glob('/dev/shm/accelerometer*') + glob.glob('/dev/shm/gyroscope*'):
            try:
                os.remove(f)
                print(f'[sensord] Removed old shm: {f}')
            except OSError:
                pass

        os.umask(0o000)

        try:
            ret = run_sensord(sample_rate=100.0, test_mode=False)
            if ret == 0:
                return  # 正常退出
            # run_sensord 返回非零 = 初始化失败 (SUID/streaming)
            print(f"[sensord] run_sensord 返回 {ret} (尝试 {attempt+1}/{MAX_RETRIES}), {delay:.0f}s 后重试...")
        except Exception as e:
            print(f"[sensord] 异常 (尝试 {attempt+1}/{MAX_RETRIES}): {e}, {delay:.0f}s 后重试...")

        time.sleep(delay)
        delay = min(delay * 1.5, MAX_DELAY)

    print(f"[sensord] 达到最大重试次数 ({MAX_RETRIES}), 退出")
    sys.exit(1)


if __name__ == '__main__':
    test_mode = '--test' in sys.argv
    rate = 100.0
    
    for i, arg in enumerate(sys.argv):
        if arg == '--rate' and i + 1 < len(sys.argv):
            rate = float(sys.argv[i + 1])
    
    sys.exit(run_sensord(sample_rate=rate, test_mode=test_mode))
