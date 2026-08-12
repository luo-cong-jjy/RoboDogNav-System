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

"""Pure policy for bounded collision-triggered SCAN replanning."""


RECOVERY_EXHAUSTED_PREFIX = 'RECOVERY_BUDGET_EXHAUSTED;'
RECOVERY_UNAVAILABLE_PREFIX = 'RECOVERY_UNAVAILABLE;'


def recovery_budget_exhausted(diagnostic: str) -> bool:
    """Return whether the guard has safely stopped a spent recovery episode."""
    return str(diagnostic).strip().startswith(RECOVERY_EXHAUSTED_PREFIX)


def recovery_replan_required(diagnostic: str) -> bool:
    """Return whether a stopped trajectory needs a fresh SCAN plan."""
    text = str(diagnostic).strip()
    return text.startswith(
        (RECOVERY_EXHAUSTED_PREFIX, RECOVERY_UNAVAILABLE_PREFIX)
    )


def collision_replan_available(attempts: int, maximum: int) -> bool:
    """Return whether another in-goal SCAN replan is within its hard limit."""
    return max(0, int(attempts)) < max(0, int(maximum))
