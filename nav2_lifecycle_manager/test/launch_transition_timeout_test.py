#! /usr/bin/env python3
# Copyright (c) 2026 Open Navigation LLC
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
Bring up a lifecycle manager with a deliberately short `transition_timeout`.

The managed node's configure transition takes far longer than that, so the manager
has to give up and report the failure. See test_transition_timeout.cpp.
"""

import os
import sys

from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_testing.legacy import LaunchTestService


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_slow_test',
                output='screen',
                parameters=[
                    {'use_sim_time': False},
                    {'autostart': False},
                    {'bond_timeout': 0.0},
                    # Far below the node's configure duration, so the bound is what
                    # decides the outcome rather than the transition finishing.
                    {'transition_timeout': 1.0},
                    {'node_names': ['slow_lifecycle_node']},
                ],
            ),
        ]
    )


def main(argv=sys.argv[1:]):
    ld = generate_launch_description()

    testExecutable = os.getenv('TEST_EXECUTABLE', '')

    test1_action = ExecuteProcess(
        cmd=[testExecutable], name='test_transition_timeout_gtest', output='screen'
    )

    lts = LaunchTestService()
    lts.add_test_action(ld, test1_action)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)


if __name__ == '__main__':
    sys.exit(main())
