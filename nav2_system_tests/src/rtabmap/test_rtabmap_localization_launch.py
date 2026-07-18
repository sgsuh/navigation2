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
Test `bringup_launch.py localization:=rtabmap` in localization mode.

This is the mode that replaces AMCL: RTAB-Map loads a database built earlier
and re-uses it read-only, publishing the stored map and the map -> odom
transform without extending the graph.

The database comes from test_rtabmap, which is declared as a CTest fixture, so
this test only runs after mapping has succeeded. The synthetic robot is the
same one, described in mock_robot.py, started at the same pose the mapping run
started from -- passed to RTAB-Map as `initial_pose` since there is no AMCL to
converge from a guess.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.legacy import LaunchTestService

sys.path.append(os.path.dirname(__file__))
from mock_robot import (  # noqa: E402,I100,I202
    database_path, mock_robot_nodes, START_X, START_Y,
)


def main(argv=sys.argv[1:]):
    testExecutable = os.getenv('TEST_EXECUTABLE', '')
    bringup_dir = get_package_share_directory('nav2_bringup')

    run_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'localization': 'rtabmap',
            'rtabmap_mode': 'localization',
            'rtabmap_db': database_path(),
            # Deliberately no '-d': the whole point is to re-use the database.
            'initial_pose': f'{START_X} {START_Y} 0 0 0 0',
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
        name='test_rtabmap_localization_node',
        output='screen',
    )

    lts = LaunchTestService()
    lts.add_test_action(ld, test1_action)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)


if __name__ == '__main__':
    sys.exit(main())
