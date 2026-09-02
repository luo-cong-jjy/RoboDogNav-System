# ============================================================================
# 【中文注释】本文件职责说明（中文注释工程新增，原代码未改动）
# 文件名：basic_server_protocol.py
# 所属：m20_nav2_locomotion —— M20 运动控制（仓库导航 → Deep Robotics M20 RL 控制器）
# 核心职责：官方 M20 ``basic_server`` API 的编解码器与指令辅助函数。
#   - APDU 报文构造：SYNC 同步头 + 16 字节小端序头（长度/帧号/格式）+ JSON 体；
#   - ApduStreamDecoder：从 TCP/UDP 字节流中增量解析出完整 JSON 帧；
#   - 解析函数：BasicStatus(1002/6)、MotionStatus(1002/4)、设备错误(1002/3)；
#   - apply_vendor_velocity_envelope：应用官方非零速度区间（V1.1.7+），
#     'zero' 策略把低于区间下限的指令安全归零，绝不放大指令。
# 本模块不依赖 ROS，可直接在无机器/无网络环境下单元测试。
# ============================================================================
# Copyright 2026 Virdyn Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Codec and command helpers for the official M20 ``basic_server`` API.

The implementation follows the APDU and JSON structures in the M20 Pro
technical manuals shipped under ``src/third_party/山猫M20_相关技术手册``.
Keeping this module free of ROS dependencies makes the wire contract directly
unit-testable without a robot or a network connection.
"""
# 【中文注释】模块说明：官方 M20 basic_server API 的编解码器与指令辅助函数。
# 实现遵循 M20 Pro 技术手册（位于 src/third_party/山猫M20_相关技术手册）中的
# APDU 与 JSON 结构；本模块无 ROS 依赖，便于直接单元测试。

from dataclasses import dataclass   # 数据类装饰器
from datetime import datetime       # 日期时间（报文时间戳）
import json                         # JSON 编解码
import math                         # 数学库：isfinite 等
import struct                       # 二进制打包/解包（小端序头）
from typing import Any, Dict, Iterable, List, Optional, Tuple  # 类型提示


SYNC = b'\xeb\x91\xeb\x90'   # 【中文注释】APDU 同步头（4 字节固定魔数）
HEADER_SIZE = 16             # 【中文注释】APDU 头长度（16 字节）
JSON_FORMAT = 0x01           # 【中文注释】数据格式标识：JSON
MAX_ASDU_SIZE = 65535        # 【中文注释】ASDU 负载最大字节数（65535）


def make_patrol_message(
    message_type: int,
    command: int,
    items: Optional[Dict[str, Any]] = None,
    timestamp: Optional[datetime] = None,
) -> Dict[str, Any]:
    """Create the common JSON ASDU envelope required by ``basic_server``."""
    # 【中文注释】构造 basic_server 要求的通用 JSON ASDU 信封（PatrolDevice）。
    # 参数：message_type 消息类型（如 2=运动控制/1002=状态上报）；command 命令号；
    #   items 业务负载字典；timestamp 时间戳（默认当前时间）。
    # 返回：PatrolDevice 信封字典。
    current = timestamp or datetime.now().astimezone()
    # Velocity examples include milliseconds, while other examples use whole
    # seconds. Milliseconds are accepted by the documented parser and preserve
    # useful ordering information for the 20 Hz command stream.
    # 【中文注释】速度示例包含毫秒，其他示例用整秒；毫秒被文档解析器接受，
    # 且为 20Hz 指令流保留了有用的排序信息。
    time_text = current.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]  # 截断到毫秒
    return {
        'PatrolDevice': {
            'Type': int(message_type),
            'Command': int(command),
            'Time': time_text,
            'Items': dict(items or {}),
        }
    }


def encode_json_apdu(message: Dict[str, Any], frame_id: int) -> bytes:
    """Encode one JSON object into the documented 16-byte APDU header."""
    # 【中文注释】把一个 JSON 对象编码为带 16 字节文档化 APDU 头的报文。
    # 参数：message 待编码的 JSON 信封；frame_id 帧号（低 16 位）。
    # 返回：完整 APDU 字节串（头 + JSON 体）。
    body = json.dumps(  # 紧凑 JSON 序列化（无多余空格，非 ASCII 保留）
        message,
        ensure_ascii=False,
        separators=(',', ':'),
    ).encode('utf-8')
    if len(body) > MAX_ASDU_SIZE:
        raise ValueError('basic_server JSON payload exceeds 65535 bytes')
    header = (  # 16 字节头：同步字 + 长度(小端 u16) + 帧号(小端 u16) + 格式 + 7 字节保留
        SYNC
        + struct.pack('<H', len(body))
        + struct.pack('<H', int(frame_id) & 0xFFFF)
        + bytes((JSON_FORMAT,))
        + bytes(7)
    )
    return header + body


class ApduStreamDecoder:
    """Incrementally decode APDUs from TCP or UDP byte streams."""
    # 【中文注释】APDU 流解码器：从 TCP/UDP 字节流中增量解码 APDU。

    def __init__(self) -> None:
        # 【中文注释】构造函数：初始化内部字节缓冲。
        self._buffer = bytearray()

    def feed(self, chunk: bytes) -> List[Tuple[int, int, Dict[str, Any]]]:
        """Return all complete JSON frames found after appending ``chunk``."""
        # 【中文注释】追加 chunk 后返回其中所有完整的 JSON 帧。
        # 返回：[(帧号, 数据格式, JSON字典), ...]。
        self._buffer.extend(chunk)
        decoded: List[Tuple[int, int, Dict[str, Any]]] = []
        while True:
            sync_at = self._buffer.find(SYNC)  # 找同步头
            if sync_at < 0:
                # Retain a possible partial sync prefix for the next read.
                # 【中文注释】保留可能的半截同步前缀供下次读取。
                keep = min(len(self._buffer), len(SYNC) - 1)
                if keep:
                    self._buffer[:] = self._buffer[-keep:]
                else:
                    self._buffer.clear()
                break
            if sync_at:  # 丢弃同步头之前的垃圾字节
                del self._buffer[:sync_at]
            if len(self._buffer) < HEADER_SIZE:  # 头未齐：等待更多数据
                break
            asdu_size = struct.unpack_from('<H', self._buffer, 4)[0]  # 负载长度
            total_size = HEADER_SIZE + asdu_size
            if len(self._buffer) < total_size:  # 整帧未齐：等待更多数据
                break
            frame_id = struct.unpack_from('<H', self._buffer, 6)[0]  # 帧号
            data_format = self._buffer[8]  # 数据格式
            body = bytes(self._buffer[HEADER_SIZE:total_size])
            del self._buffer[:total_size]  # 消费该帧
            if data_format != JSON_FORMAT:  # 非 JSON 格式帧跳过
                continue
            try:
                value = json.loads(body.decode('utf-8'))  # 解析 JSON 体
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue  # 坏帧跳过
            if isinstance(value, dict):
                decoded.append((frame_id, data_format, value))
        return decoded


@dataclass(frozen=True)  # 【中文注释】冻结数据类
class BasicStatus:
    """Subset of Type=1002, Command=6 needed by the backend state machine."""
    # 【中文注释】Type=1002, Command=6 基本状态报文中，后端状态机所需的字段子集。

    motion_state: int    # 运动状态（如 17 = RL 控制）
    gait: int            # 当前步态
    control_usage_mode: int  # 控制使用模式
    oaa_state: int       # OAA（自主避障）状态
    hard_estop: int      # 硬急停状态（0=未触发，1=触发，-1=未知）
    version: str         # 固件版本号


@dataclass(frozen=True)  # 【中文注释】冻结数据类
class MotionStatus:
    """Measured planar motion from Type=1002, Command=4."""
    # 【中文注释】Type=1002, Command=4 的实测平面运动。

    linear_x: float   # 实测前向速度（m/s）
    linear_y: float   # 实测横向速度（m/s）
    omega_z: float    # 实测偏航角速度（rad/s）
    height: float     # 机体高度（m）


def _patrol_device(message: Dict[str, Any]) -> Dict[str, Any]:
    # 【中文注释】从消息中取出 PatrolDevice 子字典（非字典时返回空字典）。
    patrol = message.get('PatrolDevice', {})
    return patrol if isinstance(patrol, dict) else {}


def parse_basic_status(message: Dict[str, Any]) -> Optional[BasicStatus]:
    """Parse a documented Type=1002, Command=6 BasicStatus report."""
    # 【中文注释】解析文档化的 Type=1002, Command=6 基本状态报告；非该报文返回 None。
    patrol = _patrol_device(message)
    if patrol.get('Type') != 1002 or patrol.get('Command') != 6:
        return None
    items = patrol.get('Items', {})
    basic = items.get('BasicStatus', {}) if isinstance(items, dict) else {}
    if not isinstance(basic, dict):
        return None
    try:
        return BasicStatus(
            motion_state=int(basic['MotionState']),
            gait=int(basic['Gait']),
            control_usage_mode=int(basic.get('ControlUsageMode', -1)),
            oaa_state=int(basic.get('OAA', 0)),
            # HES is the physical tail rotary emergency-stop state.  Keep -1
            # as an explicit unavailable value so callers can fail closed.
            # 【中文注释】HES 是机尾旋钮式物理急停状态；用 -1 表示显式不可用，
            # 使调用方可安全地"故障闭锁"。
            hard_estop=int(basic.get('HES', -1)),
            version=str(basic.get('Version', '')),
        )
    except (KeyError, TypeError, ValueError):
        return None  # 字段缺失/类型错误 → 解析失败


def parse_motion_status(message: Dict[str, Any]) -> Optional[MotionStatus]:
    """Parse Type=1002, Command=4 measured body velocity."""
    # 【中文注释】解析 Type=1002, Command=4 的实测机体速度；非该报文返回 None。
    patrol = _patrol_device(message)
    if patrol.get('Type') != 1002 or patrol.get('Command') != 4:
        return None
    items = patrol.get('Items', {})
    motion = items.get('MotionStatus', {}) if isinstance(items, dict) else {}
    if not isinstance(motion, dict):
        return None
    try:
        return MotionStatus(
            linear_x=float(motion['LinearX']),
            linear_y=float(motion['LinearY']),
            omega_z=float(motion['OmegaZ']),
            height=float(motion.get('Height', 0.0)),
        )
    except (KeyError, TypeError, ValueError):
        return None


def parse_error(message: Dict[str, Any]) -> Optional[Tuple[int, str]]:
    """Return a command reply's ``ErrorCode`` and text when present."""
    # 【中文注释】当指令应答中存在 ErrorCode 时返回 (错误码, 错误文本)；否则返回 None。
    patrol = _patrol_device(message)
    items = patrol.get('Items', {})
    if not isinstance(items, dict) or 'ErrorCode' not in items:
        return None
    try:
        return int(items['ErrorCode']), str(items.get('ErrorMessage', ''))
    except (TypeError, ValueError):
        return None


@dataclass(frozen=True)  # 【中文注释】冻结数据类
class DeviceError:
    """One asynchronous Type=1002, Command=3 hardware error item."""
    # 【中文注释】一条异步 Type=1002, Command=3 硬件错误项。

    code: int            # 错误码
    component: str       # 出错部件/设备名
    message: str         # 错误描述


def parse_device_errors(
    message: Dict[str, Any],
) -> Optional[Tuple[DeviceError, ...]]:
    """
    Parse the factory asynchronous device-error report.

    ``None`` means the message is not a device-error report.  An empty tuple
    is an explicit report with no active errors.
    """
    # 【中文注释】解析工厂异步设备错误报告。
    # None 表示该消息不是设备错误报告；空元组表示显式报告但无活动错误。
    patrol = _patrol_device(message)
    if patrol.get('Type') != 1002 or patrol.get('Command') != 3:
        return None
    items = patrol.get('Items', {})
    if not isinstance(items, dict):
        return ()
    raw_errors = items.get('ErrorList', [])
    if isinstance(raw_errors, dict):  # 单个错误字典归一化为列表
        raw_errors = [raw_errors]
    if not isinstance(raw_errors, list):
        return ()
    parsed = []
    for raw in raw_errors:
        if not isinstance(raw, dict):
            continue
        try:
            code = int(raw.get('ErrorCode', 0))
        except (TypeError, ValueError):
            continue
        if code == 0:  # 错误码 0 = 无错误，跳过
            continue
        parsed.append(
            DeviceError(
                code=code,
                component=str(
                    raw.get('Component', raw.get('DeviceName', 'unknown'))
                ),
                message=str(
                    raw.get('ErrorMessage', raw.get('ErrorMsg', ''))
                ),
            )
        )
    return tuple(parsed)


# Official non-zero velocity intervals for software package >= V1.1.7.
# Values are (minimum magnitude, maximum magnitude) for x, y and yaw.
# 【中文注释】软件包 >= V1.1.7 的官方非零速度区间表。
# 每个步态的值为 (最小幅值, 最大幅值)，分别对应 x、y、yaw 三个轴。
GAIT_VELOCITY_INTERVALS = {
    0x1001: ((0.20, 2.00), (0.35, 1.00), (0.50, 2.00)),  # 步态 0x1001
    0x1003: ((0.15, 2.00), (0.30, 1.00), (0.40, 2.00)),  # 步态 0x1003
    0x3002: ((0.15, 2.00), (0.25, 1.00), (0.35, 1.50)),  # 步态 0x3002（敏捷平地步态）
    0x3003: ((0.15, 2.00), (0.30, 1.00), (0.40, 2.00)),  # 步态 0x3003
}


def apply_vendor_velocity_envelope(
    command: Iterable[float],
    gait: int,
    subthreshold_policy: str = 'zero',
    zero_epsilon: float = 1.0e-4,
) -> Tuple[Tuple[float, float, float], Tuple[str, ...]]:
    """
    Apply the official velocity intervals without defeating safety.

    ``zero`` is the safe default: an upstream command below the documented
    non-zero interval is converted to zero, never enlarged after collision
    prediction. ``passthrough`` is available only for controlled firmware
    characterization. Both policies clamp the documented maximum magnitude.
    """
    # 【中文注释】应用官方速度区间，且不破坏安全性。
    # 'zero' 为安全默认：低于文档非零区间的上游指令转为零，碰撞预测后绝不放大；
    # 'passthrough' 仅供受控的固件特性标定使用。两种策略都会把幅值夹到文档最大值。
    values = tuple(float(value) for value in command)
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError('velocity command must contain three finite values')
    policy = str(subthreshold_policy).lower()
    if policy not in {'zero', 'passthrough'}:
        raise ValueError('subthreshold_policy must be zero or passthrough')
    intervals = GAIT_VELOCITY_INTERVALS.get(  # 未知名步态回退到 0x3002 区间
        int(gait), GAIT_VELOCITY_INTERVALS[0x3002]
    )
    names = ('x', 'y', 'yaw')
    output = []
    suppressed = []
    for name, value, (minimum, maximum) in zip(names, values, intervals):
        magnitude = abs(value)
        if magnitude <= zero_epsilon:  # 近零指令保持零
            output.append(0.0)
            continue
        if magnitude < minimum and policy == 'zero':  # 低于区间下限 → 归零并记录抑制轴
            output.append(0.0)
            suppressed.append(name)
            continue
        output.append(math.copysign(min(magnitude, maximum), value))  # 夹到最大值，保留符号
    return (output[0], output[1], output[2]), tuple(suppressed)
