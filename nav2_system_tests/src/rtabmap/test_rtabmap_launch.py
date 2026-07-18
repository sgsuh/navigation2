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
End-to-end test of `bringup_launch.py localization:=rtabmap`, without a robot.

Synthetic sensor data replaces the hardware:

  map_server (ground truth)
      the depot map, reachable ONLY over the /map_server/map service and
      published on `ground_truth_map` so that it can never be confused with
      the real `map` topic that RTAB-Map owns
    -> loopback_simulator
      raycasts that map into a LaserScan and integrates cmd_vel into the
      odom->base_link TF. Its own map->odom publisher is disabled, leaving
      RTAB-Map as the single owner of that transform, exactly as on a robot
      with wheel odometry.
    -> bringup_launch.py localization:=rtabmap
      RTAB-Map never sees the ground truth; it has to rebuild the map from
      scans, publish it, and have Nav2's StaticLayer pick it up.

Runs with use_rgbd:=False because there is no synthetic camera: static fake
images would produce meaningless visual features and bogus loop closures. The
map-topic and QoS wiring under test is identical either way, but note that the
RGB-D path itself is therefore not covered here.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
from launch_testing.legacy import LaunchTestService


def main(argv=sys.argv[1:]):
    testExecutable = os.getenv('TEST_EXECUTABLE', '')
    bringup_dir = get_package_share_directory('nav2_bringup')
    ground_truth_map = os.path.join(bringup_dir, 'maps', 'depot.yaml')
    rtabmap_db = os.path.join(os.environ.get('TMPDIR', '/tmp'),
                              'test_rtabmap.db')

    run_ground_truth_map_server = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': ground_truth_map,
                     'topic_name': 'ground_truth_map'}],
    )

    run_loopback_simulator = launch_ros.actions.Node(
        package='nav2_loopback_sim',
        executable='loopback_simulator',
        name='loopback_simulator',
        output='screen',
        parameters=[{
            # RTAB-Map, not the simulator, must provide map->odom.
            'publish_map_odom_tf': False,
            'publish_clock': False,
            'base_frame_id': 'base_link',
            'scan_frame_id': 'base_scan',
            'scan_range_max': 10.0,
            'scan_publish_dur': 0.1,
        }],
    )

    # map_server is listed first: loopback_simulator pulls the ground truth
    # from its /map_server/map service while activating.
    run_lifecycle_manager = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mock',
        output='screen',
        parameters=[{'node_names': ['map_server', 'loopback_simulator']},
                    {'autostart': True}],
    )

    # The laser mounting, normally supplied by the robot's URDF.
    run_base_to_scan_tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_scan',
        output='log',
        arguments=['--x', '0.2', '--z', '0.15',
                   '--frame-id', 'base_link', '--child-frame-id', 'base_scan'],
    )

    run_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'localization': 'rtabmap',
            'rtabmap_mode': 'mapping',
            'rtabmap_db': rtabmap_db,
            'rtabmap_args': '-d',  # start from an empty database every run
            'use_rgbd': 'False',
            'use_composition': 'False',
            'use_keepout_zones': 'False',
            'use_speed_zones': 'False',
            'autostart': 'True',
        }.items(),
    )

    ld = LaunchDescription([
        run_ground_truth_map_server,
        run_loopback_simulator,
        run_lifecycle_manager,
        run_base_to_scan_tf,
        run_nav2,
    ])

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
