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
End-to-end test of `bringup_launch.py localization:=rtabmap` in mapping mode.

The synthetic robot is described in mock_robot.py. RTAB-Map never sees the
ground truth map: it has to rebuild it from scans, publish it, and have Nav2's
StaticLayer pick it up.

On success this leaves a populated database at mock_robot.database_path(),
which is the fixture test_rtabmap_localization consumes.

Runs with use_rgbd:=False because there is no synthetic camera: static fake
images would produce meaningless visual features and bogus loop closures. The
map-topic and QoS wiring under test is identical either way. The RGB-D path is
covered by test_rtabmap_visual, which runs in Gazebo for that reason.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.legacy import LaunchTestService

sys.path.append(os.path.dirname(__file__))
from mock_robot import database_path, mock_robot_nodes  # noqa: E402,I100,I202


def main(argv=sys.argv[1:]):
    testExecutable = os.getenv('TEST_EXECUTABLE', '')
    bringup_dir = get_package_share_directory('nav2_bringup')

    run_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'localization': 'rtabmap',
            'rtabmap_mode': 'mapping',
            'rtabmap_db': database_path(),
            'rtabmap_args': '-d',  # start from an empty database every run
            'use_rgbd': 'False',
            'use_composition': 'False',
            'use_keepout_zones': 'False',
            'use_speed_zones': 'False',
            'autostart': 'True',
        }.items(),
    )

    ld = LaunchDescription(mock_robot_nodes() + [run_nav2])

    test1_action = ExecuteProcess(
        cmd=[sys.executable, testExecutable],
        name='test_rtabmap_node',
        output='screen',
    )

    lts = LaunchTestService()
    lts.add_test_action(ld, test1_action)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)


if __name__ == '__main__':
    sys.exit(main())
