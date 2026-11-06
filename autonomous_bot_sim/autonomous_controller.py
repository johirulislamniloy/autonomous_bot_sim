#!/usr/bin/env python3
"""ROS 2 wrapper around ColorCourseNavigator.

Inputs
    /tof_range            TOF on the sweeping servo head
    /sonar_left           left  ultrasonic range (simulated)
    /sonar_right          right ultrasonic range (simulated)
    /odom                 wheel odometry
    /imu/data             gyro/IMU (used to straighten the body)
    /front_camera/image_raw   angled-down camera (obstacle colours + line)
Outputs
    /cmd_vel
    /set_joint_trajectory  servo head position

The camera is deliberately angled down (0.35 rad, see the URDF):
  * middle rows  -> red / green obstacle faces
  * bottom rows  -> cyan finish strip on the floor

The navigator does the actual course logic; this node only converts ROS
messages to its simple sensor inputs and reports the total challenge time.
"""
import math
import time
from collections import deque

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, PoseStamped, PoseArray
from sensor_msgs.msg import Range, Image, Imu, JointState
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from gazebo_msgs.srv import SetEntityState

from autonomous_bot_sim.color_navigator import ColorCourseNavigator

# ---------- settings ----------
SERVO_JOINT = 'servo_joint'
LOOP_PERIOD = 0.05            # 20 Hz
SONAR_MAX = 1.20              # URDF sonar max range
VISION_TIMEOUT = 0.50         # older camera data = finish not seen
FINISH_PIX = 25               # cyan pixels in the bottom rows

# Camera is tilted down 0.35 rad at 0.185 m height.
#   finish band: rows 88% .. 100% (floor strip right in front)
FIN_ROW_LO = 0.88

# Gazebo sensor/odom publishers use BEST_EFFORT. Subscribers must match.
SENSOR_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST, depth=5)
# Images are large: depth 1 so a slow frame can never build a backlog.
IMAGE_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=1)
CMD_QOS = QoSProfile(depth=10)
# The obstacle layout is published once and must survive a late subscriber.
LATCH_QOS = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)


class AutonomousController(Node):
    def __init__(self):
        super().__init__('autonomous_controller')
        self.nav = ColorCourseNavigator()

        # A re-entrant group keeps one entity from locking every other callback.
        # Without it, a malformed Gazebo sample leaves rclpy's default
        # mutually-exclusive group permanently "executing" (the exception is
        # raised before ending_execution()), which freezes the whole node.
        self.cb_group = ReentrantCallbackGroup()

        self.pub = self.create_publisher(Twist, '/cmd_vel', CMD_QOS)
        self.path_pub = self.create_publisher(Path, '/trail', 1)
        self.servo_pub = self.create_publisher(JointTrajectory,
                                               '/set_joint_trajectory', CMD_QOS)
        self.set_state_cli = self.create_client(SetEntityState,
                                                '/set_entity_state')

        self.create_subscription(Odometry, '/odom', self.odom_cb, SENSOR_QOS,
                                 callback_group=self.cb_group)
        self.create_subscription(Bool, '/start_button', self.start_cb, CMD_QOS,
                                 callback_group=self.cb_group)

        # --- simulated ultrasonic side sonars ---
        self.sonar_left = SONAR_MAX
        self.sonar_right = SONAR_MAX
        self.create_subscription(Range, '/sonar_left', self.left_cb, SENSOR_QOS,
                                 callback_group=self.cb_group)
        self.create_subscription(Range, '/sonar_right', self.right_cb, SENSOR_QOS,
                                 callback_group=self.cb_group)

        # --- camera: cyan finish strip only (box colour is never used) ---
        self.finish_px = 0
        self.vision_t = -1.0
        self.create_subscription(Image, '/front_camera/image_raw',
                                 self.color_cb, IMAGE_QOS,
                                 callback_group=self.cb_group)

        # --- TOF on the servo head ---
        self.have_tof = False
        self.create_subscription(Range, '/tof_range', self.tof_cb, SENSOR_QOS,
                                 callback_group=self.cb_group)

        # --- box layout published by obstacle_randomizer ---
        # Stored in world coordinates; converted to the navigator frame once
        # the exact start pose is known.
        self.raw_obstacles = []
        self.start_x = 0.0
        self.start_y = -1.35
        self.start_yaw = math.pi / 2.0
        self.create_subscription(PoseArray, '/obstacle_poses',
                                 self.obstacle_cb, LATCH_QOS,
                                 callback_group=self.cb_group)

        # --- odometry / IMU ---
        self.have_odom = False
        self.x = self.y = self.yaw = 0.0
        self.have_imu = False
        self.imu_yaw = self.imu_yaw0 = 0.0
        self.create_subscription(Imu, '/imu/data', self.imu_cb, SENSOR_QOS,
                                 callback_group=self.cb_group)

        # Dead-reckoned pose for the navigator.  Wheel odometry integrates
        # position with the measured heading, so every in-place turn (and the
        # wheel slip it causes) leaves a permanent x/y offset.  Instead we
        # integrate the *commanded* forward speed along the IMU heading: an
        # in-place turn has v=0, so it can no longer shift the estimated
        # position, while straights (where the wheels really do roll) stay
        # accurate.  The navigator's own frame starts at (0, 0, 0) with the
        # start heading along +x.
        self.dr_x = self.dr_y = 0.0
        self.dr_t = None
        self.last_v_cmd = 0.0

        # actual servo angle over time (to tag every TOF beam angle exactly)
        self.servo_hist = deque(maxlen=256)
        self.create_subscription(JointState, '/joint_states', self.joint_cb,
                                 SENSOR_QOS, callback_group=self.cb_group)

        self.state = 'IDLE'          # IDLE -> ALIGNING -> ALIGN_WAIT -> RUNNING -> FINISHED
        self.align_future = None
        self.align_done_t = None
        self.align_request_t = None
        self.start_t_abs = None
        self.finish_t_abs = None
        self.path = Path()
        self.path.header.frame_id = 'odom'
        self.last_path_t = 0.0

        self.create_timer(LOOP_PERIOD, self.control_loop,
                          callback_group=self.cb_group)
        self.get_logger().info(
            'Controller ready (servo+TOF, 2 side sonars, angled camera). '
            'Waiting for START...')

    # ------------------------------------------------------------- callbacks
    def left_cb(self, msg):
        self.sonar_left = self._clean_range(msg)

    def right_cb(self, msg):
        self.sonar_right = self._clean_range(msg)

    @staticmethod
    def _clean_range(msg):
        r = float(msg.range)
        if not math.isfinite(r):
            return SONAR_MAX
        if r <= 0.0 or r >= msg.max_range:
            return SONAR_MAX
        return min(r, SONAR_MAX)

    def joint_cb(self, msg):
        if SERVO_JOINT not in msg.name:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # joint states can arrive very slightly out of order; keep it sorted
        if self.servo_hist and t < self.servo_hist[-1][0]:
            self.servo_hist.append((t, msg.position[msg.name.index(SERVO_JOINT)]))
            ordered = sorted(self.servo_hist)
            self.servo_hist.clear()
            self.servo_hist.extend(ordered)
        else:
            self.servo_hist.append((t, msg.position[msg.name.index(SERVO_JOINT)]))

    def beam_angle_at(self, t_msg):
        """Servo angle at a TOF message stamp (linear interpolation)."""
        hist = self.servo_hist
        if not hist:
            return None
        i = len(hist) - 1
        while i > 0 and hist[i][0] > t_msg:
            i -= 1
        t1, a1 = hist[i]
        if t1 > t_msg:
            return None
        if i == len(hist) - 1:
            return a1
        t2, a2 = hist[i + 1]
        if t2 <= t1:
            return a1
        f = (t_msg - t1) / (t2 - t1)
        return a1 + f * (a2 - a1)

    def tof_cb(self, msg):
        r = float(msg.range)
        if not math.isfinite(r) or r > msg.max_range:
            r = float(msg.max_range)
        if r <= 0.0:
            return
        t_msg = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        a = self.beam_angle_at(t_msg)
        if a is None:
            return
        self.nav.tof(a, r, t_msg)
        self.have_tof = True

    def obstacle_cb(self, msg):
        """Cache the exact box layout for the local path planner."""
        self.raw_obstacles = [(float(p.position.x), float(p.position.y))
                              for p in msg.poses]
        if self.state == 'RUNNING':
            self._send_obstacles()

    def _send_obstacles(self):
        """Convert world box centres into the navigator (progress, lateral)
        frame and hand them to the planner."""
        pts = []
        c, s = math.cos(self.start_yaw), math.sin(self.start_yaw)
        for wx, wy in self.raw_obstacles:
            dx, dy = wx - self.start_x, wy - self.start_y
            pts.append((dx * c + dy * s, -dx * s + dy * c))
        self.nav.set_obstacles(pts)

    def odom_cb(self, msg):
        p = msg.pose.pose
        self.x, self.y = float(p.position.x), float(p.position.y)
        q = p.orientation
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.have_odom = True

    def imu_cb(self, msg):
        q = msg.orientation
        self.imu_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.have_imu = True

    def heading(self):
        if self.have_imu:
            return math.atan2(math.sin(self.imu_yaw - self.imu_yaw0),
                              math.cos(self.imu_yaw - self.imu_yaw0))
        return math.atan2(math.sin(self.yaw - self.nav.yaw0),
                          math.cos(self.yaw - self.nav.yaw0))

    def color_cb(self, msg):
        """Detect the cyan finish strip (obstacle colours are ignored)."""
        if msg.encoding not in ('rgb8', 'bgr8'):
            return
        d = msg.data
        swap = msg.encoding == 'bgr8'
        h, w, step = msg.height, msg.width, msg.step

        fin_lo = int(h * FIN_ROW_LO)
        cyan = 0
        for j in range(fin_lo, h, 2):
            base = j * step
            for i in range(0, w, 2):
                p = base + 3 * i
                r, g, b = d[p], d[p + 1], d[p + 2]
                if swap:
                    r, b = b, r
                if g > 55 and b > 55 and r < 0.70 * g and r < 0.70 * b \
                        and abs(g - b) < 100:
                    cyan += 1

        self.finish_px = cyan
        self.vision_t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.vision_t <= 0.0:
            self.vision_t = self.get_clock().now().nanoseconds / 1e9

    def start_cb(self, msg):
        if not msg.data or self.state != 'IDLE':
            return
        if not self.have_odom or not self.have_tof:
            self.get_logger().warn(
                'Odometry/TOF data not ready yet, cannot start.')
            return
        # Put the bot back on the start line before timing begins.  Gazebo
        # lets it creep ~0.1 m while the contacts settle, which would offset
        # every progress-based stop.  This is asynchronous; control_loop
        # finishes the start once the service answers and the IMU has settled.
        if self.set_state_cli.service_is_ready():
            req = SetEntityState.Request()
            req.state.name = 'autonomous_bot'
            req.state.pose.position.x = 0.0
            req.state.pose.position.y = -1.35
            req.state.pose.position.z = 0.005
            req.state.pose.orientation.z = 0.7071068
            req.state.pose.orientation.w = 0.7071068
            self.align_future = self.set_state_cli.call_async(req)
            self.align_request_t = self.get_clock().now().nanoseconds / 1e9
            self.state = 'ALIGNING'
            self.get_logger().info('START requested: aligning on the start line...')
            return
        self._begin_run()

    def _begin_run(self):
        # The navigator works in its own lane frame: (0, 0) at the start pose,
        # heading 0 toward the finish.  Dead reckoning replaces the drifting
        # wheel-odometry x/y (see __init__).
        self.imu_yaw0 = self.imu_yaw
        self.dr_x = self.dr_y = 0.0
        self.dr_t = self.get_clock().now().nanoseconds / 1e9
        self.last_v_cmd = 0.0
        # The launch/start teleport places the robot at world (0, -1.35) with
        # yaw +90 deg.  Use that *world* pose for the map transform; /odom is
        # published in its own (encoder) frame, so its x/y/yaw are not world
        # coordinates.
        self.start_x, self.start_y = 0.0, -1.35
        self.start_yaw = math.pi / 2.0
        self.nav.reset(0.0, 0.0, 0.0)
        self._send_obstacles()
        self.start_t_abs = self.get_clock().now().nanoseconds / 1e9
        self.state = 'RUNNING'
        self.get_logger().info(
            f'START! Heading source: {"IMU" if self.have_imu else "odom"}. '
            f'Timer started.')

    # --------------------------------------------------------------- helpers
    def _dead_reckon(self, t_abs):
        """Integrate last commanded speed along the IMU heading."""
        if self.dr_t is None:
            self.dr_t = t_abs
            return
        dt = t_abs - self.dr_t
        self.dr_t = t_abs
        if dt <= 0.0:
            return
        dt = min(dt, 0.20)              # survive a scheduler hiccup
        rel = self.heading()
        self.dr_x += self.last_v_cmd * math.cos(rel) * dt
        self.dr_y += self.last_v_cmd * math.sin(rel) * dt

    def elapsed(self):
        if self.start_t_abs is None:
            return 0.0
        return (self.get_clock().now().nanoseconds / 1e9) - self.start_t_abs

    def total_time(self):
        """Seconds from START to the stop instant (finish, or Ctrl+C)."""
        if self.start_t_abs is None:
            return 0.0
        stop = self.finish_t_abs
        if stop is None:
            stop = self.get_clock().now().nanoseconds / 1e9
        return max(0.0, stop - self.start_t_abs)

    def publish_trail(self):
        if self.elapsed() - self.last_path_t < 0.1:
            return
        self.last_path_t = self.elapsed()
        pose = PoseStamped()
        pose.header.frame_id = 'odom'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        self.path.poses.append(pose)
        if len(self.path.poses) > 3000:
            self.path.poses = self.path.poses[::2]
        self.path_pub.publish(self.path)

    # -------------------------------------------------------------- recovery
    def recover_after_bad_sample(self):
        """Un-wedge every entity after a failed take_message().

        rclpy's handler marks an entity as executing before taking the message;
        if the C++ message conversion raises, that flag is never cleared and the
        entity is never scheduled again.  Clearing it lets the next executor run
        the subscription normally.  The callback group is re-entrant, so there
        is no group lock to release here.
        """
        for entity in list(self.subscriptions) + list(self.timers):
            entity._executor_event = False

    def _finish_run(self, reason):
        self.state = 'FINISHED'
        if self.finish_t_abs is None:
            self.finish_t_abs = self.get_clock().now().nanoseconds / 1e9
        self.pub.publish(Twist())
        print('\n' + '=' * 62, flush=True)
        print('  CHALLENGE COMPLETE in %.2f seconds' % self.elapsed(), flush=True)
        print('  reason: %s' % reason, flush=True)
        print('=' * 62 + '\n', flush=True)
        self.get_logger().info(
            f'CHALLENGE COMPLETE in {self.elapsed():.2f} s ({reason})')

    # -------------------------------------------------------------- main loop
    def control_loop(self):
        t_abs = self.get_clock().now().nanoseconds / 1e9

        # servo: sweep once per scan, otherwise look straight ahead
        if self.state == 'RUNNING':
            angle = self.nav.servo_cmd(t_abs)
        else:
            angle = 0.0
        traj = JointTrajectory()
        traj.header.frame_id = 'world'
        traj.joint_names = [SERVO_JOINT]
        pt = JointTrajectoryPoint()
        pt.positions = [float(angle)]
        pt.time_from_start = Duration(seconds=0.02).to_msg()
        traj.points = [pt]
        self.servo_pub.publish(traj)

        if self.state == 'ALIGNING':
            if self.align_future is not None and self.align_future.done():
                try:
                    self.align_future.result()
                except Exception as exc:  # noqa: BLE001 - service failure
                    self.get_logger().warn(f'Start alignment failed: {exc}')
                    self._begin_run()
                    return
                self.align_done_t = t_abs
                self.state = 'ALIGN_WAIT'
            elif self.align_request_t is not None \
                    and t_abs - self.align_request_t > 2.0:
                self.get_logger().warn(
                    'Start alignment timed out; running from current pose.')
                self.align_future = None
                self._begin_run()
                return
            self.pub.publish(Twist())
            return

        if self.state == 'ALIGN_WAIT':
            # Give the freshly teleported IMU sample time to arrive so the
            # start heading (and therefore every relative turn) is exact.
            if self.align_done_t is not None and t_abs - self.align_done_t >= 0.50:
                self._begin_run()
            self.pub.publish(Twist())
            return

        if self.state != 'RUNNING':
            # Actively hold the wheels still: the diff-drive plugin otherwise
            # lets the freshly spawned bot creep a few cm before START, which
            # would offset the dead-reckoned finish target.
            self.pub.publish(Twist())
            return

        self.publish_trail()
        if self.nav.finished:
            self._finish_run(self.nav.finish_reason or 'finish line reached')
            return

        # Update the drift-free pose with the speed commanded on the previous
        # cycle, then hand it to the navigator.
        self._dead_reckon(t_abs)

        fresh = (t_abs - self.vision_t) < VISION_TIMEOUT
        finish = fresh and self.finish_px >= FINISH_PIX

        v, w = self.nav.step(x=self.dr_x, y=self.dr_y, rel=self.heading(),
                             t=t_abs,
                             sonar_left=self.sonar_left,
                             sonar_right=self.sonar_right,
                             finish=finish)

        if self.nav.finished:
            self._finish_run(self.nav.finish_reason or 'finish line reached')
            return

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(w)
        self.pub.publish(cmd)
        self.last_v_cmd = float(v)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousController()

    # rclpy's global spin() can wedge when take_message() fails to convert one
    # malformed Gazebo sensor sample (a known rclpy/Gazebo race, most often on
    # the high-rate /tof_range topic).  Drive a private executor with a short
    # timeout instead: a bad sample is dropped, a warning is printed, and the
    # very next iteration keeps the run alive.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    last_warn = 0.0
    try:
        while rclpy.ok():
            try:
                executor.spin_once(timeout_sec=0.05)
            except ExternalShutdownException:
                break
            except RuntimeError as exc:
                now = time.monotonic()
                if now - last_warn > 1.0:
                    import traceback
                    node.get_logger().warning(
                        f'Dropped one malformed sensor sample: {exc}\n'
                        + traceback.format_exc())
                    last_warn = now
                # A failed take can leave the old executor's task table
                # wedged.  Abandon it (do NOT call shutdown(), which waits for
                # the stuck task), clear the per-entity "executing" flags and
                # give the node a clean executor.
                node.recover_after_bad_sample()
                executor = SingleThreadedExecutor()
                executor.add_node(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Ctrl+C can invalidate the rclpy context before this block runs, so
        # stop the motors defensively and always print the run summary.
        try:
            total = node.total_time()
        except (Exception, KeyboardInterrupt):  # context may be shut down
            total = None
        finished = node.state == 'FINISHED'
        try:
            node.pub.publish(Twist())
        except (Exception, KeyboardInterrupt):  # context may be shut down
            pass
        if node.start_t_abs is not None:
            label = ('TOTAL RUN TIME (start -> finish)' if finished
                     else 'TOTAL RUN TIME (start -> exit)')
            value = total if total is not None else 0.0
            print('\n' + '=' * 62, flush=True)
            print('  %s: %.2f seconds' % (label, value), flush=True)
            print('=' * 62 + '\n', flush=True)
        try:
            executor.shutdown()
        except (Exception, KeyboardInterrupt):  # context may be shut down
            pass
        try:
            node.destroy_node()
        except (Exception, KeyboardInterrupt):  # a late Ctrl+C is harmless
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except (Exception, KeyboardInterrupt):
            pass


if __name__ == '__main__':
    main()
