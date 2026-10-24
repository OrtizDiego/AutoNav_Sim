# Copyright 2026 AutoNav Team
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

"""Nav2 speed limits agree with each other and with the security guard."""

import os

import yaml

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _load(name):
    with open(os.path.join(PKG_DIR, 'config', name)) as f:
        return yaml.safe_load(f)


def test_speed_limits_agree():
    nav2 = _load('nav2_params.yaml')
    dwb = nav2['controller_server']['ros__parameters']['FollowPath']
    smoother = nav2['velocity_smoother']['ros__parameters']
    guard = _load('behavior_params.yaml')['security_guard_bt']['ros__parameters']
    person = _load('behavior_params.yaml')['person_controller']['ros__parameters']

    speed = dwb['max_vel_x']
    assert dwb['max_speed_xy'] == speed
    assert smoother['max_velocity'][0] == speed      # else the smoother clips it
    assert smoother['min_velocity'][0] == -speed
    assert smoother['max_velocity'][2] == dwb['max_vel_theta']
    # Nav2's search can gain on a walking intruder; direct follow stays faster
    assert person['walk_speed'] < speed <= guard['max_linear_speed']


def test_local_costmap_covers_the_trajectories():
    nav2 = _load('nav2_params.yaml')
    dwb = nav2['controller_server']['ros__parameters']['FollowPath']
    local = nav2['local_costmap']['local_costmap']['ros__parameters']
    reach = dwb['max_vel_x'] * dwb['sim_time']
    assert reach < min(local['width'], local['height']) / 2.0
