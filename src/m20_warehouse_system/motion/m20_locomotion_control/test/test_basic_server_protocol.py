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

from datetime import datetime, timezone
import json
import struct

import pytest

from m20_locomotion_control.basic_server_protocol import (
    ApduStreamDecoder,
    HEADER_SIZE,
    SYNC,
    apply_vendor_velocity_envelope,
    encode_json_apdu,
    make_patrol_message,
    parse_basic_status,
    parse_device_errors,
    parse_error,
    parse_motion_status,
)


def _status_message(command, key, value):
    return {
        'PatrolDevice': {
            'Type': 1002,
            'Command': command,
            'Time': '2026-08-10 12:00:00.000',
            'Items': {key: value},
        }
    }


def test_json_apdu_uses_documented_little_endian_header():
    message = make_patrol_message(
        2,
        25,
        {'X': 0.2, 'Y': 0.0, 'Yaw': -0.4},
        datetime(2026, 8, 10, 12, 0, tzinfo=timezone.utc),
    )
    packet = encode_json_apdu(message, 0x1234)

    assert packet[:4] == SYNC
    assert struct.unpack_from('<H', packet, 4)[0] == len(packet) - HEADER_SIZE
    assert struct.unpack_from('<H', packet, 6)[0] == 0x1234
    assert packet[8] == 1
    assert packet[9:HEADER_SIZE] == bytes(7)
    assert json.loads(packet[HEADER_SIZE:].decode('utf-8')) == message


def test_stream_decoder_handles_fragmentation_and_junk_prefix():
    first = encode_json_apdu(make_patrol_message(100, 100), 7)
    second_message = make_patrol_message(2, 22, {'MotionParam': 1})
    second = encode_json_apdu(second_message, 8)
    decoder = ApduStreamDecoder()

    assert decoder.feed(b'junk' + first[:11]) == []
    decoded = decoder.feed(first[11:] + second)

    assert [frame[0] for frame in decoded] == [7, 8]
    assert decoded[1][2] == second_message


def test_status_and_error_parsers_follow_official_fields():
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
    assert (basic.motion_state, basic.gait) == (17, 12290)
    assert basic.control_usage_mode == 1
    assert basic.hard_estop == 0
    assert motion is not None
    assert motion.linear_x == pytest.approx(0.42)
    assert motion.linear_y == pytest.approx(-0.12)
    assert motion.omega_z == pytest.approx(0.31)
    assert parse_error(
        {
            'PatrolDevice': {
                'Items': {'ErrorCode': 3, 'ErrorMessage': 'rejected'}
            }
        }
    ) == (3, 'rejected')


def test_device_error_parser_handles_factory_async_report():
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
                {'ErrorCode': 0, 'Component': 'ignored'},
            ],
        )
    )

    assert errors is not None
    assert len(errors) == 1
    assert errors[0].code == 101
    assert errors[0].component == 'joint_motor'
    assert parse_device_errors(make_patrol_message(100, 100)) is None


def test_vendor_envelope_never_enlarges_subthreshold_safe_command():
    output, suppressed = apply_vendor_velocity_envelope(
        (0.10, -0.20, 0.20),
        gait=0x3002,
    )

    assert output == (0.0, 0.0, 0.0)
    assert suppressed == ('x', 'y', 'yaw')


def test_vendor_envelope_clamps_maximum_and_supports_characterization():
    clamped, _ = apply_vendor_velocity_envelope(
        (3.0, -2.0, 4.0), gait=0x3002
    )
    passthrough, suppressed = apply_vendor_velocity_envelope(
        (0.10, -0.20, 0.20),
        gait=0x3002,
        subthreshold_policy='passthrough',
    )

    assert clamped == (2.0, -1.0, 1.5)
    assert passthrough == (0.10, -0.20, 0.20)
    assert suppressed == ()


@pytest.mark.parametrize(
    'command',
    [(1.0, 2.0), (float('nan'), 0.0, 0.0)],
)
def test_vendor_envelope_rejects_invalid_commands(command):
    with pytest.raises(ValueError):
        apply_vendor_velocity_envelope(command, gait=0x3002)
