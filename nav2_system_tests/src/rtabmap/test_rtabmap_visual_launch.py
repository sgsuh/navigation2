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
Covers RTAB-Map's RGB-D path, which the other two RTAB-Map tests cannot.

They run on `nav2_loopback_sim`, which synthesises a LaserScan and odometry but
no images, so both use `use_rgbd:=False`. This one needs a rendered camera, so
it runs in Gazebo, following the same pattern as
`src/system/test_system_with_obstacle_launch.py`.

Choices worth knowing about:

* **turtlebot4, not the waffle.** tb4's standard description already carries an
  `rgbd_camera` sensor and `spawn_tb4.launch.py` already bridges colour, depth
  and camera_info. The tb3 waffle's camera is depth-only and its bridge omits
  the camera entirely, so using it would mean committing a modified copy of the
  500-line waffle SDF plus a bridge launch into this package.
* **A world built here, not tb3_sandbox or depot.** tb3_sandbox contains no
  texture images at all and yields almost no visual features. depot works well
  but is a 101 MB Fuel download, i.e. a network dependency at test time that no
  other test in this package has. `worlds/rtabmap_textured_room.sdf` gets its
  features from geometry and shading instead, with no external assets.
* **`rtabmap_launch.py` directly, not `bringup_launch.py`.** The costmap and
  map-topic wiring is already asserted by `test_rtabmap`; adding the whole Nav2
  stack here would only contribute unrelated lifecycle failure modes to a test
  about a sensor path.

Requires a working GL stack, since Gazebo has to render. Software rendering is
enough (`LIBGL_ALWAYS_SOFTWARE=1`); this runs in real time at 320x240.
"""

import os
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchService
from launch.actions import (AppendEnvironmentVariable, ExecuteProcess,
                            IncludeLaunchDescription)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_testing.legacy import LaunchTestService

# Middle of the room, clear of the crates lining the walls. The tester drives a
# racetrack from here that stays inside x [-1.5, 1.3], y [-1.5, 0.0].
START_X, START_Y = '-1.5', '-1.5'


def main(argv=sys.argv[1:]):
    testExecutable = os.getenv('TEST_EXECUTABLE', '')
    tests_dir = get_package_share_directory('nav2_system_tests')
    sim_dir = get_package_share_directory('nav2_minimal_tb4_sim')
    desc_dir = get_package_share_directory('nav2_minimal_tb4_description')

    world = os.path.join(tests_dir, 'worlds', 'rtabmap_textured_room.sdf')
    robot_xacro = os.path.join(desc_dir, 'urdf', 'standard', 'turtlebot4.urdf.xacro')
    database = os.path.join(os.environ.get('TMPDIR', '/tmp'),
                            'test_rtabmap_visual.db')
    # Mapping must start from scratch or a stale database from a previous run
    # would supply the loop closures this test is trying to observe.
    if os.path.exists(database):
        os.remove(database)

    gazebo = ExecuteProcess(cmd=['gz', 'sim', '-r', '-s', world], output='screen')

    # spawn_tb4.launch.py creates the model from the robot_description topic
    # rather than from a file, so robot_state_publisher has to be up first.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command(['xacro', ' ', robot_xacro]),
        }],
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_dir, 'launch', 'spawn_tb4.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'x_pose': START_X,
            'y_pose': START_Y,
            'z_pose': '0.01',
            'roll': '0.0',
            'pitch': '0.0',
            'yaw': '0.0',
        }.items(),
    )

    run_rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'),
                         'launch', 'rtabmap_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'rtabmap_mode': 'mapping',
            'rtabmap_db': database,
            'use_rgbd': 'True',
            # tb4 publishes the camera under /rgbd_camera, not the
            # realsense-style names rtabmap_launch.py defaults to.
            'rgb_topic': '/rgbd_camera/image',
            'depth_topic': '/rgbd_camera/depth_image',
            'camera_info_topic': '/rgbd_camera/camera_info',
        }.items(),
    )

    ld = LaunchDescription([
        # The tb4 description refers to its meshes as
        # package://nav2_minimal_tb4_description/..., so Gazebo needs the
        # directory *containing* that share directory on its resource path.
        # Without it the robot's own visuals fail to load -- harmless for a
        # camera pointed outward, but 15 lines of [Err] in every run.
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH', str(Path(desc_dir).parent.resolve())),
        gazebo,
        robot_state_publisher,
        spawn_robot,
        run_rtabmap,
    ])

    test1_action = ExecuteProcess(
        cmd=[sys.executable, testExecutable],
        name='test_rtabmap_visual_node',
        output='screen',
    )

    lts = LaunchTestService()
    lts.add_test_action(ld, test1_action)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)


if __name__ == '__main__':
    sys.exit(main())
