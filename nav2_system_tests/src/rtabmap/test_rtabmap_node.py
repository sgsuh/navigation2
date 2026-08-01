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

"""Assert the RTAB-Map -> Nav2 contract; see test_rtabmap_launch.py."""

import math
import sys

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener

# What nav2's StaticLayer subscribes with, and what rtabmap publishes with
# (its `latch` parameter defaults to true).
LATCHED = QoSProfile(depth=1,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
SENSOR = QoSProfile(depth=10,
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE)

# The robot starts here, near the middle of depot.yaml. That map has origin
# [0, 0] and spans (0,0)..(30.2, 15.4); loopback_simulator hands back an
# all-inf scan for any pose on or outside the map border, so the start pose
# has to be well inside it.
START_X, START_Y = 15.0, 7.8


class RtabmapTester(Node):

    def __init__(self) -> None:
        super().__init__('rtabmap_tester')
        self.scans: list[LaserScan] = []
        self.maps: list[OccupancyGrid] = []
        self.costmaps: list[Costmap] = []

        self.initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.create_subscription(LaserScan, 'scan', self.scans.append, SENSOR)
        self.create_subscription(OccupancyGrid, 'map', self.maps.append, LATCHED)
        self.create_subscription(
            Costmap, 'global_costmap/costmap_raw', self.costmaps.append, LATCHED)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def spin(self, seconds: float) -> None:
        end = self.get_clock().now().nanoseconds + seconds * 1e9
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for(self, predicate, timeout: float, what: str) -> bool:
        """Spin until predicate() holds. Returns False on timeout."""
        end = self.get_clock().now().nanoseconds + timeout * 1e9
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            if predicate():
                return True
            rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().error(f'timed out after {timeout:.0f}s waiting for {what}')
        return False

    def valid_scan_beams(self) -> int:
        """Count finite returns in the newest scan, 0 if there is none."""
        if not self.scans:
            return 0
        return sum(1 for r in self.scans[-1].ranges
                   if not math.isinf(r) and not math.isnan(r))

    def publish_initial_pose(self) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = START_X
        msg.pose.pose.position.y = START_Y
        msg.pose.pose.orientation.w = 1.0
        msg.header.stamp = self.get_clock().now().to_msg()
        self.initialpose_pub.publish(msg)

    def drive(self, lin: float, ang: float, seconds: float) -> None:
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        end = self.get_clock().now().nanoseconds + seconds * 1e9
        while rclpy.ok() and self.get_clock().now().nanoseconds < end:
            self.cmd_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)


def main() -> int:
    rclpy.init()
    node = RtabmapTester()
    failures: list[str] = []

    # 1. The mock itself has to be alive before anything is asserted. A scan
    #    with finite returns is the single signal that loopback_simulator has
    #    the ground truth map, an initial pose, and the base->laser transform;
    #    until all three hold it publishes an all-inf scan instead of failing.
    #
    #    The initial pose has to be pumped from the *first* wait, not after it.
    #    loopback_simulator creates its scan timer inside the /initialpose
    #    callback and nothing else in this launch publishes that topic, so a wait
    #    for the first scan that sends no pose cannot ever be satisfied: it used
    #    to burn its full 60 s on every run, passing or failing, which was most
    #    of this test's runtime.
    ticks = 0

    def pump_initial_pose() -> None:
        # Throttled, because RTAB-Map subscribes to the same topic and logs a
        # warning for every pose it gets in mapping mode.
        nonlocal ticks
        if ticks % 20 == 0:
            node.publish_initial_pose()
        ticks += 1

    def got_a_scan() -> bool:
        pump_initial_pose()
        return len(node.scans) > 0

    def scan_is_valid() -> bool:
        pump_initial_pose()
        return node.valid_scan_beams() >= 10

    if not node.wait_for(got_a_scan, 60.0, 'the first LaserScan'):
        failures.append('the mock published no LaserScan at all')

    if not failures and not node.wait_for(
            scan_is_valid, 60.0, 'a LaserScan with finite returns'):
        failures.append(
            'the mock never produced a usable scan, so nothing downstream '
            'could be tested')

    # 2. Drive far enough to add several nodes to the graph. RGBD/LinearUpdate
    #    is 0.2 m in the mapping tuning, so this is many nodes over.
    if not failures:
        node.get_logger().info('driving...')
        for _ in range(2):
            node.drive(0.25, 0.0, 5.0)
            node.drive(0.0, 0.5, 3.0)
        node.drive(0.0, 0.0, 0.5)

        # 3. RTAB-Map must have built a grid from those scans.
        node.wait_for(
            lambda: any(sum(1 for c in m.data if c >= 65) >= 50
                        for m in node.maps[-1:]),
            60.0, 'an occupancy grid from RTAB-Map')
        if not node.maps:
            failures.append('RTAB-Map published no OccupancyGrid on `map`')
        else:
            grid = node.maps[-1]
            occupied = sum(1 for c in grid.data if c >= 65)
            free = sum(1 for c in grid.data if 0 <= c < 65)
            print(f'[map] {grid.info.width}x{grid.info.height} @ '
                  f'{grid.info.resolution:.3f}m frame={grid.header.frame_id} '
                  f'occupied={occupied} free={free}')
            if occupied < 50:
                failures.append(
                    f"RTAB-Map's grid has almost no occupied cells "
                    f'({occupied}); it is not mapping the scans')
            if free < 200:
                failures.append(
                    f"RTAB-Map's grid has almost no free space ({free})")
            if grid.header.frame_id != 'map':
                failures.append(
                    f'grid frame_id is {grid.header.frame_id!r}, not "map"')

        # 4. RTAB-Map owns map->odom, the transform AMCL would otherwise supply.
        try:
            node.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
            print('[tf] map->odom present')
        except Exception as e:  # noqa: BLE001 - reported, not handled
            failures.append(f'RTAB-Map published no map->odom TF: {e}')

        # 5. The point of the whole exercise: Nav2's StaticLayer has to consume
        #    that grid. A non-rolling global costmap resizes itself to whatever
        #    StaticLayer hands it, so matching dimensions prove the map came
        #    from RTAB-Map. Counting lethal cells would not: the ObstacleLayer
        #    marks cells straight from `scan` and would pass even with the
        #    StaticLayer completely mis-wired.
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
                30.0, 'global_costmap to resize to the RTAB-Map grid')
            costmap = node.costmaps[-1]
            got = (costmap.metadata.size_x, costmap.metadata.size_y)
            print(f'[costmap] {got[0]}x{got[1]} vs grid {expected[0]}x{expected[1]}')
            if matched:
                print('[costmap] StaticLayer consumed the RTAB-Map grid')
            else:
                failures.append(
                    f"global_costmap is {got[0]}x{got[1]} but RTAB-Map's grid "
                    f'is {expected[0]}x{expected[1]}; StaticLayer did not take '
                    'the map (check the topic namespace and QoS)')

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
