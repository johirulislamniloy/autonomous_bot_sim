#!/usr/bin/env python3
"""Manual obstacle placement helper.

Two jobs:

1. Keep ``/obstacle_poses`` in sync with the boxes that are actually in
   Gazebo.  In manual mode (``randomize:=false`` in the launch file) this is
   what lets the navigator plan around boxes you place by hand: insert a box
   in the Gazebo GUI (or spawn one yourself) and the next ``/model_states``
   update is forwarded to the planner.

2. Optionally spawn a fixed list of boxes from a YAML file while the physics
   is paused, so a chosen layout is reproducible.

Examples
--------
Manual mode, boxes placed with the Gazebo GUI::

    ros2 run autonomous_bot_sim manual_obstacles

Spawn a chosen layout and resume::

    ros2 run autonomous_bot_sim manual_obstacles \
        --ros-args -p layout_file:=$HOME/my_layout.yaml

YAML format (``color`` is optional, red by default)::

    obstacles:
      - {x: -0.20, y:  0.10, color: red}
      - {x:  0.20, y:  0.55, color: green}
      - {x:  0.00, y: -0.20, color: red}
"""
import os

import rclpy
import yaml
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import Bool
from std_srvs.srv import Empty

from autonomous_bot_sim.arena_layout import OBS_SIZE, OBS_HEIGHT

# Latched: the controller may start after the layout is published.
LATCH_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)

COLOR_RGB = {
    'red':   (0.85, 0.08, 0.08),
    'green': (0.08, 0.80, 0.10),
}

# Models that are part of the arena or the robot, never obstacles.
IGNORE_PREFIXES = ('ground', 'sun', 'wall', 'arena', 'autonomous_bot')
# Names that count as a user-placed obstacle.  Gazebo GUI "Insert Box"
# produces "unit_box", "unit_box_0", ...; the randomizer uses "obstacle_N".
OBSTACLE_PREFIXES = ('obstacle', 'unit_box', 'box')


class ManualObstacles(Node):
    def __init__(self):
        super().__init__('manual_obstacles')
        self.declare_parameter('layout_file', '')
        self.declare_parameter('resume_after_spawn', True)
        self.declare_parameter('keep_paused', False)

        self.pose_pub = self.create_publisher(PoseArray, '/obstacle_poses',
                                              LATCH_QOS)
        self.ready_pub = self.create_publisher(Bool, '/obstacles_ready',
                                               LATCH_QOS)
        self.create_subscription(ModelStates, '/model_states',
                                 self.model_cb, 10)
        self.create_timer(0.5, self.publish_layout)

        self.model_names = []
        self.model_poses = []
        self.last_published = None

    # ----------------------------------------------------------------- utils
    @staticmethod
    def _is_obstacle(name):
        n = (name or '').lower()
        if n.startswith(IGNORE_PREFIXES):
            return False
        return n.startswith(OBSTACLE_PREFIXES)

    def model_cb(self, msg):
        self.model_names = list(msg.name)
        self.model_poses = list(msg.pose)

    def publish_layout(self):
        pts = [(float(p.position.x), float(p.position.y))
               for n, p in zip(self.model_names, self.model_poses)
               if self._is_obstacle(n)]
        # Avoid re-publishing an identical latched message every 0.5 s.
        key = tuple(sorted((round(x, 4), round(y, 4)) for x, y in pts))
        if key == self.last_published:
            return
        self.last_published = key
        layout = PoseArray()
        layout.header.frame_id = 'world'
        layout.header.stamp = self.get_clock().now().to_msg()
        for x, y in pts:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = OBS_HEIGHT / 2.0
            pose.orientation.w = 1.0
            layout.poses.append(pose)
        self.pose_pub.publish(layout)
        self.get_logger().info(
            'Manual mode: published %d obstacle poses on /obstacle_poses'
            % len(pts))

    # --------------------------------------------------------------- services
    def _call(self, srv_type, name, request=None, timeout=8.0):
        client = self.create_client(srv_type, name)
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error('service %s not available' % name)
            return None
        future = client.call_async(request if request is not None
                                   else srv_type.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.exception() is not None:
            self.get_logger().error('service %s failed: %s'
                                    % (name, future.exception()))
            return None
        return future.result()

    def _sdf(self, name, color):
        s, h = OBS_SIZE, OBS_HEIGHT
        r, g, b = COLOR_RGB.get(color, COLOR_RGB['red'])
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
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""

    # -------------------------------------------------------------- layout file
    def load_layout(self, path):
        path = os.path.expanduser(path)
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
        except Exception as exc:  # noqa: BLE001 - report and keep running
            self.get_logger().error('cannot read layout %s: %s' % (path, exc))
            return

        items = data.get('obstacles', []) if isinstance(data, dict) else data
        if not items:
            self.get_logger().warn('layout %s has no obstacles' % path)
            return

        self._call(Empty, '/pause_physics')

        # Remove obstacles left over from a previous run.
        for name in list(self.model_names):
            if self._is_obstacle(name):
                req = DeleteEntity.Request()
                req.name = name
                self._call(DeleteEntity, '/delete_entity', req)

        for i, item in enumerate(items, start=1):
            name = str(item.get('name', 'obstacle_%d' % i))
            color = str(item.get('color', 'red')).lower()
            req = SpawnEntity.Request()
            req.name = name
            req.xml = self._sdf(name, color)
            req.robot_namespace = ''
            req.initial_pose.position.x = float(item['x'])
            req.initial_pose.position.y = float(item['y'])
            req.initial_pose.position.z = OBS_HEIGHT / 2.0
            req.initial_pose.orientation.w = 1.0
            result = self._call(SpawnEntity, '/spawn_entity', req)
            if result is not None and result.success:
                self.get_logger().info(
                    'Spawned %s (%s) at X=%.2f, Y=%.2f'
                    % (name, color, req.initial_pose.position.x,
                       req.initial_pose.position.y))
            else:
                self.get_logger().error('failed to spawn %s' % name)

        if bool(self.get_parameter('resume_after_spawn').value) \
                and not bool(self.get_parameter('keep_paused').value):
            self._call(Empty, '/unpause_physics')
            self.get_logger().info('Physics resumed')


def main(args=None):
    rclpy.init(args=args)
    node = ManualObstacles()
    layout = str(node.get_parameter('layout_file').value)
    if layout:
        node.load_layout(layout)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
