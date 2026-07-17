#!/usr/bin/env python3
# Copyright 2026 lbw
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
"""Checks for the one-shot PTP client."""

from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / 'scripts' / 'move_to_joint_goal.py'


def test_target_and_ptp_parameters():
    source = SCRIPT.read_text(encoding='utf-8')
    assert 'TARGET = [0.0, -0.524, 0.0, -2.094, 0.0, 1.68, 0.0]' in source
    assert 'MAXIMUM_JOINT_VELOCITIES = [0.30] * 7' in source
    assert 'goal.goal_joint_configuration = TARGET' in source
    assert 'goal.maximum_joint_velocities = MAXIMUM_JOINT_VELOCITIES' in source
    assert 'TargetStatus.TARGET_REACHED' in source
