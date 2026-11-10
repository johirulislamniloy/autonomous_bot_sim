#!/usr/bin/env python3
"""Spawns randomly scattered coloured obstacles every run."""

import random
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Bool

from autonomous_bot_sim.arena_layout import (
    NUM_OBSTACLES, OBS_SIZE, OBS_HEIGHT, MIN_GAP, pick_positions, pick_colors)

# bright Gazebo materials: red = pass on the LEFT, green = pass on the RIGHT
COLOR_RGB = {
    'red':   (0.85, 0.08, 0.08),
    'green': (0.08, 0.80, 0.10),
}

# TRANSIENT_LOCAL = latched: the controller receives the flag even if it
# connects AFTER the publish (late-joiner safe).
LATCH_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)


class ObstacleRandomizer(Node):
    def __init__(self):
        super().__init__('obstacle_randomizer')
        self.declare_parameter('seed', -1)     # -1 = different layout every run
        self.declare_parameter('num_obstacles', NUM_OBSTACLES)
        self.declare_parameter('obstacle_size', OBS_SIZE)
        self.declare_parameter('obstacle_height', OBS_HEIGHT)
        self.declare_parameter('min_gap', MIN_GAP)
        self.client = self.create_client(SpawnEntity, '/spawn_entity')
        self.ready_pub = self.create_publisher(Bool, '/obstacles_ready', LATCH_QOS)
        self.pose_pub = self.create_publisher(PoseArray, '/obstacle_poses',
                                              LATCH_QOS)
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /spawn_entity service in Gazebo...')

    def make_sdf(self, name, color, size, height):
        s, h = size, height
        r, g, b = COLOR_RGB[color]
        return f"""<?xml version="1.0"?>
<sdf version="1.6">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><box><size>{s} {s} {h}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{s} {s} {h}</size></box></geometry>
        <material>
          <ambient>{r:.2f} {g:.2f} {b:.2f} 1</ambient>
          <diffuse>{r:.2f} {g:.2f} {b:.2f} 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""

    def spawn_all(self):
        seed = int(self.get_parameter('seed').value)
        n = int(self.get_parameter('num_obstacles').value)
        size = float(self.get_parameter('obstacle_size').value)
        height = float(self.get_parameter('obstacle_height').value)
        min_gap = float(self.get_parameter('min_gap').value)
        rng = random.Random(None if seed < 0 else seed)
        colors = pick_colors(rng, n)
        positions = pick_positions(rng, n, size=size, min_gap=min_gap)
        self.get_logger().info(
            f'{n} obstacles ({colors.count("red")} red, '
            f'{colors.count("green")} green), {size:.2f} m cubes, '
            f'random scatter, min gap {min_gap:.2f} m')

        futures = []
        for i, ((x, y), color) in enumerate(zip(positions, colors), start=1):
            req = SpawnEntity.Request()
            req.name = f'obstacle_{i}'
            req.xml = self.make_sdf(req.name, color, size, height)
            req.robot_namespace = ''
            # Position is set ONLY here (not in the SDF) to avoid double offset
            req.initial_pose.position.x = float(x)
            req.initial_pose.position.y = float(y)
            req.initial_pose.position.z = height / 2.0
            futures.append((req.name, x, y, color, self.client.call_async(req)))

        for name, x, y, color, future in futures:
            rclpy.spin_until_future_complete(self, future)
            result = future.result()
            if result is not None and result.success:
                self.get_logger().info(
                    f'Spawned {name} ({color}) at X={x:.2f}, Y={y:.2f}')
            else:
                self.get_logger().error(f'Failed to spawn {name}')

        # latched "arena is complete" flag for the controller
        msg = Bool()
        msg.data = True
        self.ready_pub.publish(msg)

        # Publish the exact box centres so the navigator can plan a safe path.
        layout = PoseArray()
        layout.header.frame_id = 'world'
        layout.header.stamp = self.get_clock().now().to_msg()
        for x, y in positions:
            pose = Pose()
            pose.position.x = float(x)
            pose.position.y = float(y)
            pose.position.z = height / 2.0
            pose.orientation.w = 1.0
            layout.poses.append(pose)
        self.pose_pub.publish(layout)
        self.get_logger().info(
            f'Published {len(layout.poses)} obstacle poses on /obstacle_poses')
        self.get_logger().info('All obstacles ready -> /obstacles_ready = TRUE (latched).')
        # keep spinning briefly so the latched messages are actually sent out
        end = time.monotonic() + 1.0
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleRandomizer()
    node.spawn_all()
    node.get_logger().info('All obstacles ready. Press ENTER in start_button terminal to run.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
