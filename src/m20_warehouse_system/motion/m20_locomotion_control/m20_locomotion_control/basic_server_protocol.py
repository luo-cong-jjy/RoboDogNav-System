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

from dataclasses import dataclass
from datetime import datetime
import json
import math
import struct
from typing import Any, Dict, Iterable, List, Optional, Tuple


SYNC = b'\xeb\x91\xeb\x90'
HEADER_SIZE = 16
JSON_FORMAT = 0x01
MAX_ASDU_SIZE = 65535


def make_patrol_message(
    message_type: int,
    command: int,
    items: Optional[Dict[str, Any]] = None,
    timestamp: Optional[datetime] = None,
) -> Dict[str, Any]:
    """Create the common JSON ASDU envelope required by ``basic_server``."""
    current = timestamp or datetime.now().astimezone()
    # Velocity examples include milliseconds, while other examples use whole
    # seconds. Milliseconds are accepted by the documented parser and preserve
    # useful ordering information for the 20 Hz command stream.
    time_text = current.strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
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
    body = json.dumps(
        message,
        ensure_ascii=False,
        separators=(',', ':'),
    ).encode('utf-8')
    if len(body) > MAX_ASDU_SIZE:
        raise ValueError('basic_server JSON payload exceeds 65535 bytes')
    header = (
        SYNC
        + struct.pack('<H', len(body))
        + struct.pack('<H', int(frame_id) & 0xFFFF)
        + bytes((JSON_FORMAT,))
        + bytes(7)
    )
    return header + body


class ApduStreamDecoder:
    """Incrementally decode APDUs from TCP or UDP byte streams."""

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, chunk: bytes) -> List[Tuple[int, int, Dict[str, Any]]]:
        """Return all complete JSON frames found after appending ``chunk``."""
        self._buffer.extend(chunk)
        decoded: List[Tuple[int, int, Dict[str, Any]]] = []
        while True:
            sync_at = self._buffer.find(SYNC)
            if sync_at < 0:
                # Retain a possible partial sync prefix for the next read.
                keep = min(len(self._buffer), len(SYNC) - 1)
                if keep:
                    self._buffer[:] = self._buffer[-keep:]
                else:
                    self._buffer.clear()
                break
            if sync_at:
                del self._buffer[:sync_at]
            if len(self._buffer) < HEADER_SIZE:
                break
            asdu_size = struct.unpack_from('<H', self._buffer, 4)[0]
            total_size = HEADER_SIZE + asdu_size
            if len(self._buffer) < total_size:
                break
            frame_id = struct.unpack_from('<H', self._buffer, 6)[0]
            data_format = self._buffer[8]
            body = bytes(self._buffer[HEADER_SIZE:total_size])
            del self._buffer[:total_size]
            if data_format != JSON_FORMAT:
                continue
            try:
                value = json.loads(body.decode('utf-8'))
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue
            if isinstance(value, dict):
                decoded.append((frame_id, data_format, value))
        return decoded


@dataclass(frozen=True)
class BasicStatus:
    """Subset of Type=1002, Command=6 needed by the backend state machine."""

    motion_state: int
    gait: int
    control_usage_mode: int
    oaa_state: int
    hard_estop: int
    version: str


@dataclass(frozen=True)
class MotionStatus:
    """Measured planar motion from Type=1002, Command=4."""

    linear_x: float
    linear_y: float
    omega_z: float
    height: float


def _patrol_device(message: Dict[str, Any]) -> Dict[str, Any]:
    patrol = message.get('PatrolDevice', {})
    return patrol if isinstance(patrol, dict) else {}


def parse_basic_status(message: Dict[str, Any]) -> Optional[BasicStatus]:
    """Parse a documented Type=1002, Command=6 BasicStatus report."""
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
            hard_estop=int(basic.get('HES', -1)),
            version=str(basic.get('Version', '')),
        )
    except (KeyError, TypeError, ValueError):
        return None


def parse_motion_status(message: Dict[str, Any]) -> Optional[MotionStatus]:
    """Parse Type=1002, Command=4 measured body velocity."""
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
    patrol = _patrol_device(message)
    items = patrol.get('Items', {})
    if not isinstance(items, dict) or 'ErrorCode' not in items:
        return None
    try:
        return int(items['ErrorCode']), str(items.get('ErrorMessage', ''))
    except (TypeError, ValueError):
        return None


@dataclass(frozen=True)
class DeviceError:
    """One asynchronous Type=1002, Command=3 hardware error item."""

    code: int
    component: str
    message: str


def parse_device_errors(
    message: Dict[str, Any],
) -> Optional[Tuple[DeviceError, ...]]:
    """
    Parse the factory asynchronous device-error report.

    ``None`` means the message is not a device-error report.  An empty tuple
    is an explicit report with no active errors.
    """
    patrol = _patrol_device(message)
    if patrol.get('Type') != 1002 or patrol.get('Command') != 3:
        return None
    items = patrol.get('Items', {})
    if not isinstance(items, dict):
        return ()
    raw_errors = items.get('ErrorList', [])
    if isinstance(raw_errors, dict):
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
        if code == 0:
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
GAIT_VELOCITY_INTERVALS = {
    0x1001: ((0.20, 2.00), (0.35, 1.00), (0.50, 2.00)),
    0x1003: ((0.15, 2.00), (0.30, 1.00), (0.40, 2.00)),
    0x3002: ((0.15, 2.00), (0.25, 1.00), (0.35, 1.50)),
    0x3003: ((0.15, 2.00), (0.30, 1.00), (0.40, 2.00)),
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
    values = tuple(float(value) for value in command)
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError('velocity command must contain three finite values')
    policy = str(subthreshold_policy).lower()
    if policy not in {'zero', 'passthrough'}:
        raise ValueError('subthreshold_policy must be zero or passthrough')
    intervals = GAIT_VELOCITY_INTERVALS.get(
        int(gait), GAIT_VELOCITY_INTERVALS[0x3002]
    )
    names = ('x', 'y', 'yaw')
    output = []
    suppressed = []
    for name, value, (minimum, maximum) in zip(names, values, intervals):
        magnitude = abs(value)
        if magnitude <= zero_epsilon:
            output.append(0.0)
            continue
        if magnitude < minimum and policy == 'zero':
            output.append(0.0)
            suppressed.append(name)
            continue
        output.append(math.copysign(min(magnitude, maximum), value))
    return (output[0], output[1], output[2]), tuple(suppressed)
