import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (IncludeLaunchDescription, DeclareLaunchArgument,
                            ExecuteProcess)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def generate_launch_description():
    pkg_name = 'autonomous_bot_sim'
    pkg_share = get_package_share_directory(pkg_name)

    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    world_file = os.path.join(pkg_share, 'worlds', 'arena.world')
    rviz_file = os.path.join(pkg_share, 'rviz', 'bot.rviz')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    rviz_arg = DeclareLaunchArgument('rviz', default_value='true')
    gui_arg = DeclareLaunchArgument('gui', default_value='true',
                                    description='start the Gazebo client GUI')
    seed_arg = DeclareLaunchArgument('seed', default_value='-1',
                                     description='obstacle seed (-1 = random)')
    auto_start_arg = DeclareLaunchArgument(
        'auto_start', default_value='false',
        description='publish /start_button automatically once the sim is up')
    randomize_arg = DeclareLaunchArgument(
        'randomize', default_value='true',
        description='spawn 8 random obstacles (false = place them yourself)')
    start_paused_arg = DeclareLaunchArgument(
        'start_paused', default_value='false',
        description='start Gazebo paused so you can place obstacles first')
    manual_layout_arg = DeclareLaunchArgument(
        'manual_layout', default_value='',
        description='YAML layout to spawn when randomize:=false')
    mode_arg = DeclareLaunchArgument(
        'mode', default_value='astar',
        description='Navigation mode: astar | scan | bug2')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_file,
                          'gui': LaunchConfiguration('gui'),
                          'pause': LaunchConfiguration('start_paused')}.items()
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw, 'use_sim_time': True}]
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'autonomous_bot',
                   '-x', '0.0', '-y', '-1.35', '-z', '0.01',
                   '-Y', '1.5707963'],
        output='screen'
    )

    # Gazebo classic sometimes lets the freshly spawned robot creep a few
    # centimetres while the contacts settle.  Once the bot exists, wait for
    # that transient to finish and place it exactly on the start line through
    # the ROS state interface (the `gz model` CLI hangs in this container).
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
             'until ros2 topic list 2>/dev/null | grep -q /start_button; do '
             'sleep 0.5; done; '
             'sleep 8; '
             'ros2 topic pub --once /start_button std_msgs/msg/Bool '
             '"{data: true}"'],
        condition=IfCondition(LaunchConfiguration('auto_start')),
        output='screen'
    )

    obstacle_randomizer_node = Node(
        package=pkg_name,
        executable='obstacle_randomizer',
        output='screen',
        parameters=[{'seed': LaunchConfiguration('seed')}],
        condition=IfCondition(LaunchConfiguration('randomize'))
    )

    # Manual mode: mirror the boxes currently in Gazebo onto /obstacle_poses
    # (and optionally spawn a chosen YAML layout) so the planner uses them.
    manual_obstacles_node = Node(
        package=pkg_name,
        executable='manual_obstacles',
        output='screen',
        parameters=[{'layout_file': LaunchConfiguration('manual_layout')}],
        condition=UnlessCondition(LaunchConfiguration('randomize'))
    )

    controller_node = Node(
        package=pkg_name,
        executable='autonomous_controller',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'mode': ParameterValue(
                         LaunchConfiguration('mode'), value_type=str)}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )

    # start_button ALADA terminal e chalate hobe (ENTER lage):
    #   ros2 run autonomous_bot_sim start_button
    return LaunchDescription([
        rviz_arg, gui_arg, seed_arg, auto_start_arg,
        randomize_arg, start_paused_arg, manual_layout_arg, mode_arg,
        gazebo,
        robot_state_publisher_node,
        spawn_entity,
        align_start,
        auto_start,
        obstacle_randomizer_node,
        manual_obstacles_node,
        controller_node,
        rviz_node,
    ])
