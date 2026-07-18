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

"""Assert RTAB-Map's localization mode; see test_rtabmap_localization_launch.py."""

import math
import os
import sys

from geometry_msgs.msg import PoseWithCovarianceStamped, TwistStamped
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

sys.path.append(os.path.dirname(__file__))
from mock_robot import database_path, START_X, START_Y  # noqa: E402,I100,I202

LATCHED = QoSProfile(depth=1,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
SENSOR = QoSProfile(depth=10,
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE)


class LocalizationTester(Node):

    def __init__(self) -> None:
        super().__init__('rtabmap_localization_tester')
        self.scans: list[LaserScan] = []
        self.maps: list[OccupancyGrid] = []
        self.costmaps: list[Costmap] = []

        self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel', 10)
        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10)
        self.create_subscription(LaserScan, 'scan', self.scans.append, SENSOR)
        self.create_subscription(OccupancyGrid, 'map', self.maps.append, LATCHED)
        self.create_subscription(
            Costmap, 'global_costmap/costmap_raw', self.costmaps.append, LATCHED)

        self.param_client = self.create_client(
            GetParameters, '/rtabmap/get_parameters')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def wait_for(self, predicate, timeout: float, what: str) -> bool:
        end = self.get_clock().now().nanoseconds + timeout * 1e9
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            if predicate():
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().error(f'timed out after {timeout:.0f}s waiting for {what}')
        return False

    def get_rtabmap_parameter(self, name: str, timeout: float = 30.0):
        """Read a string parameter off the rtabmap node, None if unavailable."""
        if not self.param_client.wait_for_service(timeout_sec=timeout):
            return None
        request = GetParameters.Request()
        request.names = [name]
        future = self.param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        response = future.result()
        if response is None or not response.values:
            return None
        # Every RTAB-Map parameter is string-typed; PARAMETER_NOT_SET (0) means
        # the node does not have it.
        value = response.values[0]
        if value.type != ParameterType.PARAMETER_STRING:
            return None
        return value.string_value

    def valid_scan_beams(self) -> int:
        if not self.scans:
            return 0
        return sum(1 for r in self.scans[-1].ranges
                   if not math.isinf(r) and not math.isnan(r))

    def publish_initial_pose(self) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = START_X
        msg.pose.pose.position.y = START_Y
        msg.pose.pose.orientation.w = 1.0
        self.initialpose_pub.publish(msg)

    def drive(self, lin: float, ang: float, seconds: float) -> None:
        msg = TwistStamped()
        msg.twist.linear.x = lin
        msg.twist.angular.z = ang
        end = self.get_clock().now().nanoseconds + seconds * 1e9
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)


def main() -> int:
    rclpy.init()
    node = LocalizationTester()
    failures: list[str] = []

    # 0. The fixture. Without it every later assertion would fail for a
    #    misleading reason, so say so plainly.
    db = database_path()
    if not os.path.exists(db) or os.path.getsize(db) < 10_000:
        size = os.path.getsize(db) if os.path.exists(db) else 0
        failures.append(
            f'the database fixture at {db} is missing or trivial ({size} B); '
            'test_rtabmap should have produced it')

    if not failures:
        # 1. Start the mock. loopback_simulator creates its scan and odom
        #    timers inside the /initialpose callback, so nothing at all is
        #    published until a pose arrives -- RTAB-Map would just sit there
        #    reporting that it received no data. RTAB-Map subscribes to the
        #    same topic and, unlike in mapping mode, accepts the pose, which
        #    is also how the robot is placed in the stored map.
        ticks = 0

        def scan_is_valid() -> bool:
            nonlocal ticks
            if ticks % 20 == 0:
                node.publish_initial_pose()
            ticks += 1
            return node.valid_scan_beams() >= 10

        if not node.wait_for(scan_is_valid, 60.0,
                             'a LaserScan with finite returns'):
            failures.append(
                'the mock never produced a usable scan, so nothing downstream '
                'could be tested')

    if not failures:
        # 2. The configuration actually selected localization mode. This is the
        #    one bit that distinguishes the two rtabmap_mode values, and it
        #    comes from rtabmap_localization.yaml being loaded rather than
        #    rtabmap_mapping.yaml.
        incremental = node.get_rtabmap_parameter('Mem/IncrementalMemory')
        print(f'[mode] Mem/IncrementalMemory={incremental!r}')
        if incremental is None:
            failures.append(
                'could not read Mem/IncrementalMemory off the rtabmap node')
        elif str(incremental).lower() != 'false':
            failures.append(
                f'Mem/IncrementalMemory is {incremental!r}, expected "false"; '
                'rtabmap_mode:=localization did not select '
                'rtabmap_localization.yaml')

        # 3. Nudge the robot. RTAB-Map publishes the grid from its processing
        #    loop, so the stored map does not reach the topic until something
        #    has been processed -- a standing robot publishes nothing, even
        #    with the database already loaded.
        node.drive(0.15, 0.0, 3.0)
        node.drive(0.0, 0.0, 0.5)

        # 4. A substantial map must appear. This is what shows the database is
        #    really being re-used: the graph cannot grow in localization mode
        #    (Mem/IncrementalMemory=false, asserted above), so a map with real
        #    occupied cells has nowhere to come from except the database.
        got_map = node.wait_for(
            lambda: any(sum(1 for c in m.data if c >= 65) >= 50
                        for m in node.maps[-1:]),
            60.0, 'the stored map to be published')
        if not got_map:
            failures.append(
                'RTAB-Map published no usable map; with a read-only graph that '
                'means the database was not loaded')
        else:
            grid = node.maps[-1]
            occupied = sum(1 for c in grid.data if c >= 65)
            print(f'[map] {grid.info.width}x{grid.info.height} '
                  f'occupied={occupied} (from the database)')

        # 5. RTAB-Map owns map -> odom here too; this is the transform AMCL
        #    would otherwise publish.
        if node.wait_for(
                lambda: node.tf_buffer.can_transform(
                    'map', 'odom', rclpy.time.Time()),
                30.0, 'the map->odom transform'):
            print('[tf] map->odom present')
        else:
            failures.append('RTAB-Map published no map->odom TF')

        # 6. Nav2 StaticLayer must consume the loaded map, same contract as
        #    the mapping test. See that test for why dimensions rather than
        #    lethal-cell counts.
        node.wait_for(lambda: len(node.costmaps) > 0, 60.0, 'the global costmap')
        if not node.costmaps:
            failures.append('global_costmap published nothing on costmap_raw')
        elif node.maps:
            grid = node.maps[-1]
            expected = (grid.info.width, grid.info.height)
            matched = node.wait_for(
                lambda: node.costmaps and (node.costmaps[-1].metadata.size_x,
                                           node.costmaps[-1].metadata.size_y
                                           ) == expected,
                30.0, 'global_costmap to resize to the stored map')
            got = (node.costmaps[-1].metadata.size_x,
                   node.costmaps[-1].metadata.size_y)
            print(f'[costmap] {got[0]}x{got[1]} vs map {expected[0]}x{expected[1]}')
            if matched:
                print('[costmap] StaticLayer consumed the stored map')
            else:
                failures.append(
                    f'global_costmap is {got[0]}x{got[1]} but the stored map is '
                    f'{expected[0]}x{expected[1]}; StaticLayer did not take it')

        # 7. Driving must not grow the graph. Localization mode is read-only,
        #    so the map it serves has to stay the same size no matter how far
        #    the robot goes -- in mapping mode this same drive extends it.
        #    The occupied-cell count is compared as well as the dimensions:
        #    the grid is assembled from stored node grids, which are fixed, so
        #    it must be exactly identical. Dimensions alone would be a weak
        #    check, since a short drive through already-mapped space need not
        #    enlarge the bounding box even when mapping is on.
        def grid_signature() -> tuple:
            grid = node.maps[-1]
            return (grid.info.width, grid.info.height,
                    sum(1 for c in grid.data if c >= 65))

        if node.maps:
            before = grid_signature()
            node.drive(0.25, 0.0, 5.0)
            node.drive(0.0, 0.5, 3.0)
            node.drive(0.0, 0.0, 0.5)
            node.wait_for(lambda: False, 5.0, 'post-drive map updates')
            after = grid_signature()
            print(f'[readonly] {before[0]}x{before[1]} occupied={before[2]} -> '
                  f'{after[0]}x{after[1]} occupied={after[2]}')
            if after != before:
                failures.append(
                    f'the map changed from {before} to {after} while driving; '
                    'localization mode is extending the graph instead of '
                    'staying read-only')

    print()
    if failures:
        print('FAILED:')
        for failure in failures:
            print(f'  - {failure}')
    else:
        print('ALL CHECKS PASSED')

    node.destroy_node()
    rclpy.shutdown()
    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
