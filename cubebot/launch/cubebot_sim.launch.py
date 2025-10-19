from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from sys import executable as PY
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('cubebot')
    world_path = os.path.join(pkg_share, 'worlds', 'flat.world')
    xacro_path = os.path.join(pkg_share, 'urdf', 'cubecar.urdf.xacro')

    # Process Xacro to URDF and ensure the parameter is a string
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_path]),
        value_type=str
    )

    # Gazebo Classic server (no GUI here)
    gz_server = ExecuteProcess(
        cmd=['gzserver', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Gazebo client (GUI)
    gz_client = ExecuteProcess(
        cmd=['gzclient', '--verbose'],
        output='screen'
    )

    # State publishers
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    jsp = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    # Spawn the robot from /robot_description
    spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'cubebot',
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        output='screen'
    )

    # Obstacle avoidance via python -m (bypasses missing libexec wrapper)
    avoid = ExecuteProcess(
        cmd=[
            PY, "-m", "cubebot.obstacle_avoider",
            "--ros-args",
            "-p", "linear_speed:=0.4",
            "-p", "turn_speed:=1.0",
            "-p", "min_range:=0.6",
            "-p", "fov_deg:=70.0"
        ],
        output="screen"
    )

    return LaunchDescription([
        gz_server,
        rsp,
        jsp,
        TimerAction(period=2.0, actions=[spawner]),  # give server a moment to boot
        gz_client,
        avoid
    ])
