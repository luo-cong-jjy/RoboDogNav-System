# ============================================================================
# 【中文注释】本文件职责说明（中文注释工程新增，原代码未改动）
# 文件名：test_basic_server_protocol.py
# 所属：m20_nav2_locomotion —— M20 运动控制包的单元测试
# 核心职责：测试官方 M20 ``basic_server`` 线上协议（basic_server_protocol.py）。
#   - JSON APDU 报文头（小端序长度/帧号/格式字节）；
#   - 流解码器：分片与垃圾前缀处理；
#   - 状态/运动/错误/设备错误解析器的官方字段兼容性；
#   - 厂商速度区间包络：阈值以下归零（zero）、最大幅值夹紧、passthrough 特性。
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

"""Unit tests for the official M20 ``basic_server`` wire contract."""
# 【中文注释】模块说明：官方 M20 basic_server 线上协议的单元测试。

from datetime import datetime, timezone  # 日期时间（报文时间戳）
import json                              # JSON 解析（报文体校验）
import struct                            # 二进制解包（小端序头校验）

import pytest                            # pytest 测试框架

from m20_nav2_locomotion.basic_server_protocol import (  # 被测模块
    ApduStreamDecoder,                   #   APDU 流解码器
    HEADER_SIZE,                         #   报文头长度常量
    SYNC,                                #   同步头常量
    apply_vendor_velocity_envelope,      #   厂商速度区间应用
    encode_json_apdu,                    #   JSON APDU 编码
    make_patrol_message,                 #   Patrol 消息构造
    parse_basic_status,                  #   基本状态解析
    parse_device_errors,                 #   设备错误解析
    parse_error,                         #   错误解析
    parse_motion_status,                 #   运动状态解析
)


def _status_message(command, key, value):
    # 【中文注释】构造一条 Type=1002、指定 Command 的测试报文（Items 单键）。
    return {
        'PatrolDevice': {
            'Type': 1002,
            'Command': command,
            'Time': '2026-08-10 12:00:00.000',
            'Items': {key: value},
        }
    }


def test_json_apdu_uses_documented_little_endian_header():
    # 【中文注释】校验：JSON APDU 使用文档规定的小端序报文头
    # （同步字 + 长度 + 帧号 + 格式字节 + 7 字节保留）。
    message = make_patrol_message(
        2,
        25,
        {'X': 0.2, 'Y': 0.0, 'Yaw': -0.4},
        datetime(2026, 8, 10, 12, 0, tzinfo=timezone.utc),
    )
    packet = encode_json_apdu(message, 0x1234)

    assert packet[:4] == SYNC  # 前 4 字节 = 同步头
    assert struct.unpack_from('<H', packet, 4)[0] == len(packet) - HEADER_SIZE  # 负载长度
    assert struct.unpack_from('<H', packet, 6)[0] == 0x1234  # 帧号
    assert packet[8] == 1          # 格式字节 = JSON
    assert packet[9:HEADER_SIZE] == bytes(7)  # 7 字节保留
    assert json.loads(packet[HEADER_SIZE:].decode('utf-8')) == message  # 负载可解析


def test_stream_decoder_handles_fragmentation_and_junk_prefix():
    # 【中文注释】校验：流解码器能处理分片数据与垃圾前缀。
    first = encode_json_apdu(make_patrol_message(100, 100), 7)
    second_message = make_patrol_message(2, 22, {'MotionParam': 1})
    second = encode_json_apdu(second_message, 8)
    decoder = ApduStreamDecoder()

    assert decoder.feed(b'junk' + first[:11]) == []  # 垃圾前缀 + 半帧 → 暂无完整帧
    decoded = decoder.feed(first[11:] + second)      # 补全第一帧 + 整帧第二帧

    assert [frame[0] for frame in decoded] == [7, 8]  # 按序解出两帧
    assert decoded[1][2] == second_message            # 第二帧负载正确


def test_status_and_error_parsers_follow_official_fields():
    # 【中文注释】校验：状态与错误解析器遵循官方字段（BasicStatus/MotionStatus/ErrorCode）。
    basic = parse_basic_status(
        _status_message(
            6,
            'BasicStatus',
            {
                'MotionState': 17,
                'Gait': 12290,
                'ControlUsageMode': 1,
                'OAA': 0,
                'HES': 0,
                'Version': 'V1.1.7',
            },
        )
    )
    motion = parse_motion_status(
        _status_message(
            4,
            'MotionStatus',
            {
                'LinearX': 0.42,
                'LinearY': -0.12,
                'OmegaZ': 0.31,
                'Height': 0.57,
            },
        )
    )

    assert basic is not None
    assert (basic.motion_state, basic.gait) == (17, 12290)  # RL 控制 + 敏捷平地步态
    assert basic.control_usage_mode == 1
    assert basic.hard_estop == 0               # 硬急停未触发
    assert motion is not None
    assert motion.linear_x == pytest.approx(0.42)  # 实测速度
    assert motion.linear_y == pytest.approx(-0.12)
    assert motion.omega_z == pytest.approx(0.31)
    assert parse_error(  # 错误解析
        {
            'PatrolDevice': {
                'Items': {'ErrorCode': 3, 'ErrorMessage': 'rejected'}
            }
        }
    ) == (3, 'rejected')


def test_device_error_parser_handles_factory_async_report():
    # 【中文注释】校验：设备错误解析器能处理工厂异步错误报告
    # （错误码 0 被忽略，错误码非 0 的项被保留）。
    errors = parse_device_errors(
        _status_message(
            3,
            'ErrorList',
            [
                {
                    'ErrorCode': 101,
                    'Component': 'joint_motor',
                    'ErrorMessage': 'over temperature',
                },
                {'ErrorCode': 0, 'Component': 'ignored'},  # 错误码 0 → 忽略
            ],
        )
    )

    assert errors is not None
    assert len(errors) == 1  # 仅保留一条有效错误
    assert errors[0].code == 101
    assert errors[0].component == 'joint_motor'
    assert parse_device_errors(make_patrol_message(100, 100)) is None  # 非错误报文 → None


def test_vendor_envelope_never_enlarges_subthreshold_safe_command():
    # 【中文注释】校验：厂商速度包络绝不放大阈值以下的"安全指令"
    # （0x3002 步态下 x/y/yaw 下限分别为 0.15/0.25/0.35 → 全部归零）。
    output, suppressed = apply_vendor_velocity_envelope(
        (0.10, -0.20, 0.20),
        gait=0x3002,
    )

    assert output == (0.0, 0.0, 0.0)
    assert suppressed == ('x', 'y', 'yaw')  # 三轴都被抑制


def test_vendor_envelope_clamps_maximum_and_supports_characterization():
    # 【中文注释】校验：厂商包络夹紧最大值，并支持 passthrough 特性标定模式。
    clamped, _ = apply_vendor_velocity_envelope(
        (3.0, -2.0, 4.0), gait=0x3002  # 超上限 → 夹到 (2.0, -1.0, 1.5)
    )
    passthrough, suppressed = apply_vendor_velocity_envelope(
        (0.10, -0.20, 0.20),
        gait=0x3002,
        subthreshold_policy='passthrough',  # 直通：不做归零
    )

    assert clamped == (2.0, -1.0, 1.5)
    assert passthrough == (0.10, -0.20, 0.20)
    assert suppressed == ()


@pytest.mark.parametrize(
    'command',
    [(1.0, 2.0), (float('nan'), 0.0, 0.0)],  # 非 3 元组 / 非有限值
)
def test_vendor_envelope_rejects_invalid_commands(command):
    # 【中文注释】校验：非法指令（维度不对 / 非有限值）被拒绝。
    with pytest.raises(ValueError):
        apply_vendor_velocity_envelope(command, gait=0x3002)
