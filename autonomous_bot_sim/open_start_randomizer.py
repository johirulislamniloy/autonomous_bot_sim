#!/usr/bin/env python3
"""Random start/finish pose and direction for the competition open round.

The open round is a 3 m x 3 m square ring with a 1 m wide track.  At the
event the robot is placed somewhere on the start/finish side and the driving
direction (clockwise / counter-clockwise) is assigned randomly.  This node
mirrors that behaviour in simulation:

  * picks one of the four 2 m centreline sides,
  * picks a random position along it (away from the corners),
  * picks clockwise or counter-clockwise,
  * publishes the pose and direction (latched) for the controller.

The real open round has no start/finish line, so no marker is drawn by
default (``spawn_marker:=false``).  The navigator does not use the marker:
it stops from remembered corner timing.

On the real robot this node is not needed: launch the controller with
``open_clockwise:=true`` or ``open_clockwise:=false`` instead and place the
robot on the line by hand.  The navigator itself only uses relative heading
and the side sonars, so it does not need world coordinates.
"""

import math
import random
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

LATCH_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)

# Centreline corners of the 3 x 3 m ring, traversed counter-clockwise.
CORNERS = ((-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0))
SIDE_LEN = 2.0
TRACK_HALF = 1.0


class OpenStartRandomizer(Node):
    def __init__(self):
        super().__init__('open_start_randomizer')
        self.declare_parameter('seed', -1)          # -1 = different every run
        self.declare_parameter('margin', 0.40)      # keep clear of the corners
        self.declare_parameter('track_half', TRACK_HALF)
        self.declare_parameter('line_width', 0.98)
        # The real open round has no start/finish line, so the marker is off
        # by default.  The navigator never uses it in any case: it stops from
        # remembered corner timing alone.
        self.declare_parameter('spawn_marker', False)

        self.pose_pub = self.create_publisher(PoseStamped, '/open_start_pose',
                                              LATCH_QOS)
        self.cw_pub = self.create_publisher(Bool, '/open_clockwise', LATCH_QOS)
        self.ready_pub = self.create_publisher(Bool, '/open_start_ready',
                                               LATCH_QOS)
        self.client = self.create_client(SpawnEntity, '/spawn_entity')

    # ------------------------------------------------------------- geometry
    def choose_start(self):
        seed = int(self.get_parameter('seed').value)
        rng = random.Random(None if seed < 0 else seed)
        margin = float(self.get_parameter('margin').value)
        margin = max(0.05, min(margin, SIDE_LEN / 2.0 - 0.05))

        side = rng.randrange(4)
        s = rng.uniform(margin, SIDE_LEN - margin)
        clockwise = rng.random() < 0.5

        ax, ay = CORNERS[side]
        bx, by = CORNERS[(side + 1) % 4]
        tx, ty = (bx - ax) / SIDE_LEN, (by - ay) / SIDE_LEN
        x = ax + tx * s
        y = ay + ty * s
        if clockwise:
            tx, ty = -tx, -ty

        yaw = math.atan2(ty, tx)
        # The line lies across the track, i.e. perpendicular to travel.
        line_yaw = yaw + math.pi / 2.0
        return x, y, yaw, line_yaw, clockwise, side, s

    # --------------------------------------------------------------- marker
    def make_marker_sdf(self, name, width):
        return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><box><size>{width:.3f} 0.04 0.002</size></box></geometry>
        <material>
          <ambient>0.0 0.95 1.0 1</ambient>
          <diffuse>0.0 0.95 1.0 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""

    def spawn_marker(self, x, y, line_yaw, width):
        if not self.get_parameter('spawn_marker').value:
            return
        if not self.client.wait_for_service(timeout_sec=20.0):
            self.get_logger().warn(
                'SpawnEntity service not available; start line not drawn.')
            return
        req = SpawnEntity.Request()
        req.name = 'open_start_line'
        req.xml = self.make_marker_sdf(req.name, width)
        req.robot_namespace = ''
        req.initial_pose.position.x = float(x)
        req.initial_pose.position.y = float(y)
        req.initial_pose.position.z = 0.002
        req.initial_pose.orientation.z = math.sin(line_yaw / 2.0)
        req.initial_pose.orientation.w = math.cos(line_yaw / 2.0)
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
        result = future.result()
        if result is not None and result.success:
            self.get_logger().info('Spawned the open-round start/finish line.')
        else:
            self.get_logger().warn('Could not spawn the start/finish line.')

    # ----------------------------------------------------------------- main
    def run(self):
        x, y, yaw, line_yaw, clockwise, side, s = self.choose_start()
        direction = 'clockwise' if clockwise else 'counter-clockwise'
        self.get_logger().info(
            'Open round start: side %d, %.2f m along the side, '
            'world (%.3f, %.3f), yaw %.1f deg, %s'
            % (side, s, x, y, math.degrees(yaw), direction))

        # The controller reads these before START.  Latch them so a late
        # subscriber still receives the chosen run.
        pose = PoseStamped()
        pose.header.frame_id = 'world'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.pose_pub.publish(pose)

        cw = Bool()
        cw.data = bool(clockwise)
        self.cw_pub.publish(cw)

        self.spawn_marker(x, y, line_yaw,
                          float(self.get_parameter('line_width').value))

        ready = Bool()
        ready.data = True
        self.ready_pub.publish(ready)
        self.get_logger().info(
            'Open start pose published on /open_start_pose; direction on '
            '/open_clockwise (latched).')

        # Keep spinning briefly so the latched messages leave the node.
        end = time.monotonic() + 1.0
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def main(args=None):
    rclpy.init(args=args)
    node = OpenStartRandomizer()
    node.run()
    node.get_logger().info(
        'Open round ready. Press ENTER in start_button to begin.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
