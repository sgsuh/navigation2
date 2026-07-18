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
The synthetic robot shared by the RTAB-Map system tests.

`nav2_loopback_sim` stands in for the hardware: it raycasts a ground truth map
into a LaserScan and integrates cmd_vel into the odom -> base_link transform.
Its own map -> odom publisher is disabled so that RTAB-Map is the single owner
of that transform, exactly as on a robot with wheel odometry.

The ground truth is reachable only over the /map_server/map service and is
published on `ground_truth_map`, never on `map`. RTAB-Map therefore never sees
it: it has to build the map from scans, or load it from its own database.
"""

import os

# The robot starts here, near the middle of depot.yaml. That map has origin
# [0, 0] and spans (0,0)..(30.2, 15.4); loopback_simulator returns an all-inf
# scan for any pose on or outside the map border, so the start pose has to be
# well inside it. The test nodes use the same constants.
START_X, START_Y = 15.0, 7.8

# loopback_simulator creates its scan and odom timers inside the /initialpose
# callback, so until a pose is published it emits nothing at all -- not even
# the all-inf scan it falls back to elsewhere. Both test nodes therefore have
# to publish an initial pose before anything else can be asserted.


def ground_truth_map() -> str:
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(
        get_package_share_directory('nav2_bringup'), 'maps', 'depot.yaml')


def database_path() -> str:
    """Where the mapping test writes the database the localization test reads."""
    return os.path.join(os.environ.get('TMPDIR', '/tmp'), 'test_rtabmap.db')


def mock_robot_nodes() -> list:
    """Ground truth map server, loopback simulator, and the laser mounting."""
    import launch_ros.actions

    return [
        launch_ros.actions.Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': ground_truth_map(),
                         'topic_name': 'ground_truth_map'}],
        ),
        launch_ros.actions.Node(
            package='nav2_loopback_sim',
            executable='loopback_simulator',
            name='loopback_simulator',
            output='screen',
            parameters=[{
                # RTAB-Map, not the simulator, must provide map -> odom.
                'publish_map_odom_tf': False,
                'publish_clock': False,
                'base_frame_id': 'base_link',
                'scan_frame_id': 'base_scan',
                'scan_range_max': 10.0,
                'scan_publish_dur': 0.1,
            }],
        ),
        # map_server is listed first: loopback_simulator pulls the ground truth
        # from its /map_server/map service while activating.
        launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mock',
            output='screen',
            parameters=[{'node_names': ['map_server', 'loopback_simulator']},
                        {'autostart': True}],
        ),
        # The laser mounting, normally supplied by the robot's URDF.
        launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_scan',
            output='log',
            arguments=['--x', '0.2', '--z', '0.15',
                       '--frame-id', 'base_link',
                       '--child-frame-id', 'base_scan'],
        ),
    ]
