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
Launch RTAB-Map as a drop-in replacement for both slam_toolbox and AMCL.

A single ``rtabmap_slam/rtabmap`` node covers both roles: it publishes the 2D
occupancy grid on ``map`` and broadcasts the ``map`` -> ``odom`` TF that Nav2
consumes.  Mapping vs. localization is selected by swapping the RTAB-Map ini
config (``Mem/IncrementalMemory``), not by launching a different node.

This is the wheel-odometry configuration: the robot is expected to already
publish ``odom`` -> ``<frame_id>`` TF, and RTAB-Map reads odometry from TF
(that is what setting ``odom_frame_id`` does).  No rtabmap_odom node is
launched, so ``subscribe_odom_info`` is false.

Note that rtabmap is *not* a lifecycle node, so it is deliberately absent from
any lifecycle manager's ``node_names``.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushROSNamespace, SetParameter
from nav2_common.launch import LaunchConfigAsBool


def generate_launch_description() -> LaunchDescription:
    bringup_dir = get_package_share_directory('nav2_bringup')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfigAsBool('use_sim_time')
    use_respawn = LaunchConfigAsBool('use_respawn')
    log_level = LaunchConfiguration('log_level')

    mode = LaunchConfiguration('rtabmap_mode')
    rtabmap_db = LaunchConfiguration('rtabmap_db')
    rtabmap_params_file = LaunchConfiguration('rtabmap_params_file')
    rtabmap_args = LaunchConfiguration('rtabmap_args')
    initial_pose = LaunchConfiguration('initial_pose')

    frame_id = LaunchConfiguration('frame_id')
    odom_frame_id = LaunchConfiguration('odom_frame_id')
    map_frame_id = LaunchConfiguration('map_frame_id')

    use_rgbd = LaunchConfigAsBool('use_rgbd')
    scan_topic = LaunchConfiguration('scan_topic')
    rgb_topic = LaunchConfiguration('rgb_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    camera_info_topic = LaunchConfiguration('camera_info_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    sensor_data_qos = LaunchConfiguration('sensor_data_qos')

    # Map fully qualified names to relative ones so the node's namespace can be
    # prepended.  Leaving `map` relative makes it resolve to `<namespace>/map`,
    # which is exactly where nav2_costmap_2d's StaticLayer looks for it (see
    # Layer::joinWithParentNamespace), so no costmap parameter override is needed.
    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
        ('scan', scan_topic),
        ('rgb/image', rgb_topic),
        ('depth/image', depth_topic),
        ('rgb/camera_info', camera_info_topic),
        ('odom', odom_topic),
    ]

    # Selected by rtabmap_mode unless the caller passes an explicit params file.
    default_params_file = PythonExpression(
        [
            "'",
            os.path.join(bringup_dir, 'params', 'rtabmap_'),
            "' + ('mapping' if '",
            mode,
            "' == 'mapping' else 'localization') + '.yaml'",
        ]
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes'
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level'
    )

    declare_mode_cmd = DeclareLaunchArgument(
        'rtabmap_mode', default_value='localization',
        description='RTAB-Map mode: "mapping" (SLAM, builds the database) or '
                    '"localization" (loads an existing database read-only)'
    )

    declare_rtabmap_db_cmd = DeclareLaunchArgument(
        'rtabmap_db', default_value='~/.ros/rtabmap.db',
        description='Full path to the RTAB-Map database file'
    )

    declare_rtabmap_params_file_cmd = DeclareLaunchArgument(
        'rtabmap_params_file', default_value=default_params_file,
        description='Full path to the RTAB-Map tuning parameters YAML. Defaults '
                    'to the mapping/localization file selected by rtabmap_mode'
    )

    declare_rtabmap_args_cmd = DeclareLaunchArgument(
        'rtabmap_args', default_value='--uerror',
        description='Extra RTAB-Map command line flags, e.g. "-d" to delete the '
                    'database on start'
    )

    declare_initial_pose_cmd = DeclareLaunchArgument(
        'initial_pose', default_value='',
        description='Initial pose in localization mode, as "x y z roll pitch yaw" '
                    'or "x y z qx qy qz qw"'
    )

    declare_frame_id_cmd = DeclareLaunchArgument(
        'frame_id', default_value='base_link',
        description='Robot base frame. Must match the costmaps robot_base_frame '
                    '(base_link in nav2_params.yaml); note that AMCL, which this '
                    'replaces, uses base_footprint instead'
    )

    declare_odom_frame_id_cmd = DeclareLaunchArgument(
        'odom_frame_id', default_value='odom',
        description='Odometry frame. Setting this makes RTAB-Map read odometry '
                    'from TF rather than from the odom topic'
    )

    declare_map_frame_id_cmd = DeclareLaunchArgument(
        'map_frame_id', default_value='map', description='Map frame'
    )

    declare_use_rgbd_cmd = DeclareLaunchArgument(
        'use_rgbd', default_value='True',
        description='Subscribe to the RGB-D camera for loop closure detection. '
                    'The occupancy grid itself is built from the 2D lidar '
                    '(Grid/Sensor=0 in the ini config)'
    )

    declare_scan_topic_cmd = DeclareLaunchArgument(
        'scan_topic', default_value='scan', description='2D lidar LaserScan topic'
    )

    declare_rgb_topic_cmd = DeclareLaunchArgument(
        'rgb_topic', default_value='camera/color/image_raw',
        description='RGB image topic'
    )

    declare_depth_topic_cmd = DeclareLaunchArgument(
        'depth_topic', default_value='camera/depth/image_raw',
        description='Registered depth image topic'
    )

    declare_camera_info_topic_cmd = DeclareLaunchArgument(
        'camera_info_topic', default_value='camera/color/camera_info',
        description='RGB camera info topic'
    )

    declare_odom_topic_cmd = DeclareLaunchArgument(
        'odom_topic', default_value='odom',
        description='Wheel odometry topic. Unused while odom_frame_id is set, '
                    'since odometry is then taken from TF'
    )

    declare_sensor_data_qos_cmd = DeclareLaunchArgument(
        'sensor_data_qos', default_value='2',
        description='QoS for sensor inputs: 0=system default, 1=Reliable, '
                    '2=Best Effort. Must match the sensor drivers'
    )

    load_nodes = GroupAction(
        [
            PushROSNamespace(namespace=namespace),
            SetParameter('use_sim_time', use_sim_time),
            Node(
                package='rtabmap_slam',
                executable='rtabmap',
                name='rtabmap',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[
                    rtabmap_params_file,
                    {
                        # config_path is deliberately left unset: RTAB-Map
                        # rewrites whatever ini it is given on every clean
                        # shutdown (CoreWrapper::~CoreWrapper), which would keep
                        # dirtying the file in this repo with machine-specific
                        # absolute paths. Tuning lives in rtabmap_params_file.
                        'database_path': rtabmap_db,
                        'initial_pose': initial_pose,
                        # Frames and TF. RTAB-Map owns map->odom, which is the
                        # role AMCL would otherwise play.
                        'frame_id': frame_id,
                        'odom_frame_id': odom_frame_id,
                        'map_frame_id': map_frame_id,
                        'publish_tf': True,
                        'tf_publish_period': 0.033,
                        'tf_tolerance': 0.1,
                        'wait_for_transform': 0.1,
                        'odom_tf_linear_variance': 0.001,
                        'odom_tf_angular_variance': 0.01,
                        # Sensor inputs. No rtabmap_odom node runs in this
                        # configuration, hence subscribe_odom_info is false.
                        'subscribe_scan': True,
                        'subscribe_scan_cloud': False,
                        'subscribe_depth': use_rgbd,
                        'subscribe_rgb': use_rgbd,
                        'subscribe_rgbd': False,
                        'subscribe_stereo': False,
                        'subscribe_odom_info': False,
                        'approx_sync': True,
                        'odom_sensor_sync': True,
                        'topic_queue_size': 5,
                        'sync_queue_size': 5,
                        'qos_scan': sensor_data_qos,
                        'qos_image': sensor_data_qos,
                        'qos_camera_info': sensor_data_qos,
                        'qos_odom': sensor_data_qos,
                    }
                ],
                arguments=[
                    rtabmap_args, '--ros-args', '--log-level', log_level
                ],
                remappings=remappings,
            ),
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_mode_cmd)
    ld.add_action(declare_rtabmap_db_cmd)
    ld.add_action(declare_rtabmap_params_file_cmd)
    ld.add_action(declare_rtabmap_args_cmd)
    ld.add_action(declare_initial_pose_cmd)
    ld.add_action(declare_frame_id_cmd)
    ld.add_action(declare_odom_frame_id_cmd)
    ld.add_action(declare_map_frame_id_cmd)
    ld.add_action(declare_use_rgbd_cmd)
    ld.add_action(declare_scan_topic_cmd)
    ld.add_action(declare_rgb_topic_cmd)
    ld.add_action(declare_depth_topic_cmd)
    ld.add_action(declare_camera_info_topic_cmd)
    ld.add_action(declare_odom_topic_cmd)
    ld.add_action(declare_sensor_data_qos_cmd)
    ld.add_action(load_nodes)
    return ld
