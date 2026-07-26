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
Asserts RTAB-Map recognises a revisited place from the camera.

The robot drives closed laps and RTAB-Map has to notice it has been here before.

The assertion that matters is `loop_closure_id != 0`. RTAB-Map's global loop
closure detector is appearance-based -- a Bayes filter over a bag of visual
words -- so it can only fire if images actually reach the feature extractor.
With the camera mis-wired (wrong topic, wrong QoS, no rendering, an encoding
RTAB-Map rejects) the word count goes to zero and no loop closure is ever
reported, which is precisely the failure this test exists to catch. That was
confirmed against an inverted setup: the same drive with `use_rgbd:=False`
produces 0 loop closures while lidar proximity detections are unchanged.

`proximity_detection_id` is deliberately *not* asserted on. That detector runs
off the lidar and fires either way, so it would pass with the camera dead.

The path is a racetrack -- two straights joined by two 180 degree turns -- so
that every lap returns to the start pose at the *same heading*. A revisit at a
matching viewpoint is what makes a visual loop closure possible; an out-and-back
would come home facing the wrong way.
"""

import math
import sys

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rtabmap_msgs.msg import Info
from sensor_msgs.msg import Image, LaserScan

LAPS = 3
SPEED = 0.2             # m/s
STRAIGHT = 2.0          # m
TURN_RADIUS = 0.75      # m
STOP_RANGE = 0.35       # abort if anything gets this close

# Measured on this world: 37-39 loop closures over three laps with the camera
# working, 0 with it disabled. The threshold sits far below the former and far
# above the latter, so it tolerates run-to-run variation without ever passing
# on a dead camera.
MIN_LOOP_CLOSURES = 5
MIN_WORDS_PER_FRAME = 10


class VisualLoopClosureTester(Node):

    def __init__(self) -> None:
        super().__init__('rtabmap_visual_tester')
        self.cmd = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(LaserScan, 'scan', self.on_scan,
                                 qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odom', self.on_odom,
                                 qos_profile_sensor_data)
        self.create_subscription(Image, '/rgbd_camera/image', self.on_image,
                                 qos_profile_sensor_data)
        # RTAB-Map advertises "info" relatively and runs in the root namespace,
        # so this is /info -- not /rtabmap/info.
        self.create_subscription(Info, 'info', self.on_info, 10)

        self.min_range = math.inf
        self.closest_seen = math.inf
        self.odom: Odometry = None
        self.images = 0
        self.loop_closures: list = []
        self.proximity: list = []
        self.words: list = []

    def on_scan(self, msg: LaserScan) -> None:
        finite = [r for r in msg.ranges if math.isfinite(r) and r > msg.range_min]
        self.min_range = min(finite) if finite else math.inf
        self.closest_seen = min(self.closest_seen, self.min_range)

    def on_odom(self, msg: Odometry) -> None:
        self.odom = msg

    def on_image(self, msg: Image) -> None:
        self.images += 1

    def on_info(self, msg: Info) -> None:
        if msg.loop_closure_id != 0:
            self.loop_closures.append((msg.ref_id, msg.loop_closure_id))
        if msg.proximity_detection_id != 0:
            self.proximity.append((msg.ref_id, msg.proximity_detection_id))
        stats = dict(zip(msg.stats_keys, msg.stats_values))
        words = stats.get('Keypoint/Current_frame/words')
        if words is not None:
            self.words.append(words)

    def sim_now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def wait_for(self, predicate, timeout: float, what: str) -> bool:
        deadline = self.sim_now() + timeout
        while rclpy.ok() and self.sim_now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if predicate():
                return True
        self.get_logger().error(f'timed out after {timeout} s waiting for {what}')
        return False

    def settle(self, seconds: float) -> None:
        """Keep spinning for a while, without wait_for's timeout being a failure."""
        deadline = self.sim_now() + seconds
        while rclpy.ok() and self.sim_now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def drive(self, lin: float, ang: float, seconds: float, what: str) -> bool:
        """Hold a velocity for `seconds` of sim time, aborting near an obstacle."""
        start = self.sim_now()
        while rclpy.ok() and self.sim_now() - start < seconds:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.min_range < STOP_RANGE:
                self.stop()
                self.get_logger().error(
                    f'obstacle {self.min_range:.2f} m away during {what}; '
                    'the robot has left the free area of the world')
                return False
            msg = Twist()
            msg.linear.x = lin
            msg.angular.z = ang
            self.cmd.publish(msg)
        self.stop()
        return True

    def stop(self) -> None:
        self.cmd.publish(Twist())


def main() -> int:
    rclpy.init()
    node = VisualLoopClosureTester()

    # Gazebo, the bridges and RTAB-Map all have to come up before anything can
    # be asserted. A missing camera here is the interesting failure: it usually
    # means no rendering backend rather than a broken RTAB-Map.
    if not node.wait_for(lambda: node.odom is not None, 120.0, 'odometry'):
        return 1
    if not node.wait_for(lambda: node.images > 0, 120.0,
                         'camera images (is a GL/rendering stack available?)'):
        return 1
    if not node.wait_for(lambda: len(node.words) > 0, 120.0,
                         'RTAB-Map to publish statistics'):
        return 1

    straight_s = STRAIGHT / SPEED
    omega = SPEED / TURN_RADIUS
    turn_s = math.pi / omega

    for lap in range(LAPS):
        for what, lin, ang, secs in (
            ('straight A', SPEED, 0.0, straight_s),
            ('turn 1', SPEED, omega, turn_s),
            ('straight B', SPEED, 0.0, straight_s),
            ('turn 2', SPEED, omega, turn_s),
        ):
            if not node.drive(lin, ang, secs, f'lap {lap + 1} {what}'):
                return 1
        position = node.odom.pose.pose.position
        node.get_logger().info(
            f'lap {lap + 1}/{LAPS} done, odom ({position.x:+.2f}, {position.y:+.2f}), '
            f'{len(node.loop_closures)} loop closures so far')

    node.stop()
    # Let the last few frames finish being processed.
    node.settle(3.0)

    words = sorted(node.words)
    median_words = words[len(words) // 2] if words else 0
    node.get_logger().info(
        f'{node.images} images, median {median_words:.0f} visual words/frame, '
        f'{len(node.loop_closures)} loop closures, '
        f'{len(node.proximity)} proximity detections, '
        f'closest obstacle {node.closest_seen:.2f} m')

    ok = True
    if median_words < MIN_WORDS_PER_FRAME:
        node.get_logger().error(
            f'only {median_words:.0f} visual words per frame (want >= '
            f'{MIN_WORDS_PER_FRAME}): RTAB-Map is not extracting features from '
            'the camera, so no appearance-based loop closure is possible')
        ok = False
    if len(node.loop_closures) < MIN_LOOP_CLOSURES:
        node.get_logger().error(
            f'only {len(node.loop_closures)} visual loop closures (want >= '
            f'{MIN_LOOP_CLOSURES}) after {LAPS} laps over the same path')
        ok = False
    else:
        node.get_logger().info(
            f'first loop closures: {node.loop_closures[:5]}')

    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
