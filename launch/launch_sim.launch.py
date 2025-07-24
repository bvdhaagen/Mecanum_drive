import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():
    pkg_name = 'my_bot'
    pkg_path = get_package_share_directory(pkg_name)

    # Path to xacro file
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')

    # Generate robot description
    robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_description = {'robot_description': robot_description_config}

    # Path to controller config
    controller_yaml = os.path.join(pkg_path, 'config', 'mecanum_controllers.yaml')

    
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_yaml],  
        output='screen',
    )
    
    # Gazebo simulator
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        )
    )

    # Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, **robot_description}]
    )

    # Spawn robot in Gazebo
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'my_bot'
        ],
        output='screen'
    )

    # RViz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_path, 'rviz', 'my_bot.rviz')]
    )

    # Joint State Publisher GUI
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # Spawner: Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Spawner: Mecanum Drive Controller
    mecanum_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mecanum_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    return LaunchDescription([
        controller_manager,
        gazebo,
        rsp,
        spawn,
        rviz2,
        TimerAction(
            period=3.0,
            actions=[joint_state_broadcaster_spawner]
        ),
        TimerAction(
            period=5.0,
            actions=[mecanum_controller_spawner]
        )
    ])
