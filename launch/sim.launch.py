import math
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument,
                            ExecuteProcess, OpaqueFunction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def _as_bool(value):
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _num(value, default, cast=float):
    value = str(value).strip()
    return default if value == '' else cast(value)


# Competition obstacle-round defaults.
OBS_NUM = 8
OBS_SIZE = 0.05
OBS_HEIGHT = 0.10
OBS_MIN_GAP = 0.42
OBS_FINISH_Y = 1.175
START_HOLD = 5.0

OPEN_ALIASES = ('open', 'open_round', 'open-round', 'openround',
                'ring', 'square')


def generate_launch_description():
    pkg_name = 'autonomous_bot_sim'
    pkg_share = get_package_share_directory(pkg_name)

    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    rviz_file = os.path.join(pkg_share, 'rviz', 'bot.rviz')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    gui_arg = DeclareLaunchArgument('gui', default_value='true',
                                    description='start the Gazebo client GUI')
    seed_arg = DeclareLaunchArgument('seed', default_value='-1',
                                     description='layout/start seed (-1 = random)')
    auto_start_arg = DeclareLaunchArgument(
        'auto_start', default_value='false',
        description='publish /start_button automatically once the sim is up')
    randomize_arg = DeclareLaunchArgument(
        'randomize', default_value='true',
        description='spawn random obstacles / random open-round start')
    start_paused_arg = DeclareLaunchArgument(
        'start_paused', default_value='false',
        description='start Gazebo paused so you can place obstacles first')
    manual_layout_arg = DeclareLaunchArgument(
        'manual_layout', default_value='',
        description='YAML layout to spawn when randomize:=false')
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='astar',
        description='Navigation mode: astar | scan | bug2 | open')
    obstacle_size_arg = DeclareLaunchArgument(
        'obstacle_size', default_value='',
        description='obstacle-round box side in metres (blank = 0.05)')
    obstacle_height_arg = DeclareLaunchArgument(
        'obstacle_height', default_value='',
        description='obstacle-round box height in metres (blank = 0.10)')
    num_obstacles_arg = DeclareLaunchArgument(
        'num_obstacles', default_value='',
        description='number of boxes (blank = 8)')
    min_gap_arg = DeclareLaunchArgument(
        'min_gap', default_value='',
        description='minimum edge-to-edge gap in metres (blank = 0.42)')
    finish_y_arg = DeclareLaunchArgument(
        'finish_y', default_value='',
        description='dead-reckoned finish target y (blank = 1.175)')
    start_hold_arg = DeclareLaunchArgument(
        'start_hold', default_value='',
        description='stationary hold after START in seconds (blank = 5.0)')
    open_cw_arg = DeclareLaunchArgument(
        'open_clockwise', default_value='false',
        description='open round direction for real hardware (simulation uses '
                    'the randomizer topic)')

    def launch_setup(context, *args, **kwargs):
        mode = str(LaunchConfiguration('mode').perform(context)).strip().lower()
        is_open = mode in OPEN_ALIASES

        num_obstacles = _num(
            LaunchConfiguration('num_obstacles').perform(context),
            OBS_NUM, int)
        obstacle_size = _num(
            LaunchConfiguration('obstacle_size').perform(context),
            OBS_SIZE, float)
        obstacle_height = _num(
            LaunchConfiguration('obstacle_height').perform(context),
            OBS_HEIGHT, float)
        min_gap = _num(
            LaunchConfiguration('min_gap').perform(context),
            OBS_MIN_GAP, float)
        finish_y = _num(
            LaunchConfiguration('finish_y').perform(context),
            -1.0 if is_open else OBS_FINISH_Y, float)
        start_hold = _num(
            LaunchConfiguration('start_hold').perform(context),
            START_HOLD, float)
        open_cw = _as_bool(
            LaunchConfiguration('open_clockwise').perform(context))

        world_name = 'open_round.world' if is_open else 'obstacle_round.world'
        world_file = os.path.join(pkg_share, 'worlds', world_name)

        if is_open:
            print('[launch] OPEN ROUND | %s, 3x3 m ring, 3 laps, '
                  'start_hold=%.1f s' % (world_name, start_hold), flush=True)
        else:
            print('[launch] OBSTACLE ROUND | %s | %d boxes %.2f x %.2f x '
                  '%.2f m, gap %.2f m, finish_y=%.3f, start_hold=%.1f s'
                  % (world_name, num_obstacles, obstacle_size, obstacle_size,
                     obstacle_height, min_gap, finish_y, start_hold),
                  flush=True)

        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch', 'gazebo.launch.py')),
            launch_arguments={'world': world_file,
                              'gui': LaunchConfiguration('gui'),
                              'pause': LaunchConfiguration('start_paused')}.items()
        )

        robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_raw,
                         'use_sim_time': True}]
        )

        # Obstacle round: start on the fixed yellow line.
        # Open round: a neutral point on the bottom centreline; the controller
        # snaps to the randomizer's pose on START.
        if is_open:
            spawn_x, spawn_y, spawn_yaw = 0.0, -1.0, 0.0
        else:
            spawn_x, spawn_y, spawn_yaw = 0.0, -1.35, math.pi / 2.0

        spawn_entity = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description',
                       '-entity', 'autonomous_bot',
                       '-x', '%.4f' % spawn_x,
                       '-y', '%.4f' % spawn_y,
                       '-z', '0.01',
                       '-Y', '%.7f' % spawn_yaw],
            output='screen'
        )

        # Gazebo classic sometimes lets the freshly spawned robot creep a few
        # centimetres while the contacts settle.  For the fixed obstacle start
        # line, wait for that transient and place the bot exactly on the line
        # through the ROS state interface (the `gz model` CLI hangs here).
        # The open round is teleported by the controller on START instead,
        # because its pose is only known after the randomizer has run.
        align_pose = (
            '{state: {name: autonomous_bot, '
            'pose: {position: {x: 0.0, y: -1.35, z: 0.005}, '
            'orientation: {x: 0.0, y: 0.0, z: 0.7071068, w: 0.7071068}}, '
            'twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, '
            'angular: {x: 0.0, y: 0.0, z: 0.0}}}}'
        )
        align_start = ExecuteProcess(
            cmd=['bash', '-c',
                 'until ros2 service list 2>/dev/null '
                 '| grep -q /set_entity_state; do sleep 0.5; done; '
                 'sleep 4; '
                 'ros2 service call /set_entity_state '
                 'gazebo_msgs/srv/SetEntityState "%s" >/dev/null; '
                 'sleep 0.6; '
                 'ros2 service call /set_entity_state '
                 'gazebo_msgs/srv/SetEntityState "%s" >/dev/null'
                 % (align_pose, align_pose)],
            output='screen'
        )

        # Published automatically when auto_start:=true.  The controller re-aligns
        # the robot on START, so a fixed delay after the launch is safe.
        auto_start = ExecuteProcess(
            cmd=['bash', '-c',
                 # --no-daemon: a stale ros2 daemon (common after a killed
                 # run) must not wedge the auto-start topic probe.
                 'until ros2 topic list --no-daemon 2>/dev/null '
                 '| grep -q /start_button; do sleep 0.5; done; '
                 'sleep 8; '
                 # Publish a few times: a single message can be lost while
                 # DDS discovery or the odom/TOF warm-up is still in flight.
                 'for i in 1 2 3 4 5; do '
                 'ros2 topic pub --once /start_button std_msgs/msg/Bool '
                 '"{data: true}" >/dev/null 2>&1; '
                 'sleep 0.5; done'],
            condition=IfCondition(LaunchConfiguration('auto_start')),
            output='screen'
        )

        obstacle_randomizer_node = Node(
            package=pkg_name,
            executable='obstacle_randomizer',
            output='screen',
            parameters=[{'seed': LaunchConfiguration('seed'),
                         'num_obstacles': num_obstacles,
                         'obstacle_size': obstacle_size,
                         'obstacle_height': obstacle_height,
                         'min_gap': min_gap}],
            condition=IfCondition(LaunchConfiguration('randomize'))
        )

        # Manual mode: mirror the boxes currently in Gazebo onto /obstacle_poses
        # (and optionally spawn a chosen YAML layout) so the planner uses them.
        manual_obstacles_node = Node(
            package=pkg_name,
            executable='manual_obstacles',
            output='screen',
            parameters=[{'layout_file': LaunchConfiguration('manual_layout'),
                         'obstacle_size': obstacle_size,
                         'obstacle_height': obstacle_height}],
            condition=UnlessCondition(LaunchConfiguration('randomize'))
        )

        open_start_node = Node(
            package=pkg_name,
            executable='open_start_randomizer',
            output='screen',
            parameters=[{'seed': LaunchConfiguration('seed')}],
            condition=IfCondition(LaunchConfiguration('randomize'))
        )

        controller_node = Node(
            package=pkg_name,
            executable='autonomous_controller',
            output='screen',
            parameters=[{'use_sim_time': True,
                         'mode': ParameterValue(
                             LaunchConfiguration('mode'), value_type=str),
                         'obs_size': obstacle_size,
                         'finish_y': finish_y,
                         'start_hold': start_hold,
                         'open_clockwise': open_cw}]
        )

        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_file],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen'
        )

        actions = [gazebo,
                   robot_state_publisher_node,
                   spawn_entity]
        if is_open:
            actions.append(open_start_node)
        else:
            actions.extend([align_start,
                            obstacle_randomizer_node,
                            manual_obstacles_node])
        actions.extend([auto_start, controller_node, rviz_node])
        return actions

    return LaunchDescription([
        rviz_arg, gui_arg, seed_arg, auto_start_arg,
        randomize_arg, start_paused_arg, manual_layout_arg, mode_arg,
        obstacle_size_arg, obstacle_height_arg,
        num_obstacles_arg, min_gap_arg, finish_y_arg, start_hold_arg,
        open_cw_arg,
        OpaqueFunction(function=launch_setup),
    ])
