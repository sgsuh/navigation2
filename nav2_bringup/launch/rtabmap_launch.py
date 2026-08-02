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
import sqlite3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, GroupAction, LogInfo, OpaqueFunction,
)
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushROSNamespace, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import LaunchConfigAsBool


def check_localization_database(context, *args, load_nodes=None, **kwargs):
    """Gate the RTAB-Map node on localization mode having a map to localize against.

    Returns `load_nodes` when the database is usable and a shutdown instead when
    it is not. The node is returned from here rather than added alongside this
    check so that a rejected run never starts the process at all -- an
    EmitEvent(Shutdown) added next to the node still lets it spawn and be killed
    a moment later.

    RTAB-Map does not complain about this. Pointed at a path that does not
    exist it creates a fresh database and comes up in localization mode against
    an empty graph -- measured: it logs `Localization mode
    (Mem/IncrementalMemory=false)` like any healthy run and leaves a 106 KB file
    behind. Nothing is ever published on `map`, so nav2's StaticLayer stays
    empty and `map -> odom` is never corrected, and the only symptom is a robot
    that cannot navigate. A database that exists but holds no nodes behaves the
    same way. A zero-byte file is the one case RTAB-Map does catch, aborting
    with `no such table: Node`, which is at least legible but still a crash.

    Failing here instead costs one sqlite query and names the actual problem.
    The check is deliberately one-sided: anything it cannot read confidently is
    allowed through, so a schema change upstream degrades to today's behaviour
    rather than blocking a working setup.
    """
    if context.perform_substitution(LaunchConfiguration('rtabmap_mode')) != 'localization':
        return [load_nodes]

    path = os.path.expanduser(
        context.perform_substitution(LaunchConfiguration('rtabmap_db')))

    if not os.path.exists(path):
        problem = 'does not exist'
    elif os.path.getsize(path) == 0:
        problem = 'is empty (0 bytes)'
    else:
        try:
            con = sqlite3.connect(f'file:{path}?mode=ro', uri=True)
            try:
                nodes = con.execute('SELECT count(*) FROM Node').fetchone()[0]
            finally:
                con.close()
        except sqlite3.Error:
            # Unreadable or an unfamiliar schema: not confidently wrong, so let
            # RTAB-Map have it and report whatever it finds.
            return [load_nodes]
        if nodes:
            return [load_nodes]
        problem = 'contains no map nodes'

    return [
        LogInfo(msg=f'rtabmap_launch: refusing to start localization mode -- '
                    f'the database {path} {problem}. Localization needs a map '
                    f'built by a mapping run; RTAB-Map would otherwise come up '
                    f'silently against an empty graph and publish no map at '
                    f'all. Map first with rtabmap_mode:=mapping rtabmap_args:=-d, '
                    f'or point rtabmap_db at an existing database.'),
        EmitEvent(event=Shutdown(reason='no RTAB-Map database to localize against')),
    ]


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
        description='Robot base frame. RTAB-Map looks odom -> frame_id up to get its '
                    'odometry, so this must name the frame the robot publishes odometry '
                    'against. bringup_launch.py feeds its robot_base_frame argument in '
                    'here, which keeps it equal to the costmaps robot_base_frame'
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
                        # Upstream's own default (rtabmap.launch.py). At the 0.1
                        # this used to carry, the first scan after bringup can
                        # reach RTAB-Map before odom->base_link spans the scan
                        # interval, and convertScanMsg() fails. That failure is
                        # now merely a dropped frame, but it is still noise.
                        'wait_for_transform': 0.2,
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
                        # Without a camera there are no visual features, so
                        # RTAB-Map disables bag-of-words and every node becomes
                        # a "bad signature". Leaving Mem/BadSignaturesIgnored
                        # true would then silently discard every node -- mapping
                        # produces nothing at all, with no error, just WM=0.
                        # value_type=str is required: every RTAB-Map parameter
                        # is string-typed, and launch would otherwise infer
                        # "true"/"false" as a bool and abort the node.
                        'Mem/BadSignaturesIgnored': ParameterValue(
                            PythonExpression(
                                ["'true' if ", use_rgbd, " else 'false'"]),
                            value_type=str),
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
    # load_nodes is returned by the check rather than added here, so a rejected
    # database never starts the node at all.
    ld.add_action(OpaqueFunction(
        function=check_localization_database, kwargs={'load_nodes': load_nodes}))
    return ld
