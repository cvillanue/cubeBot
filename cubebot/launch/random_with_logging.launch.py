from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cubebot',
            executable='random_policy',
            name='random_policy',
            output='screen',
            parameters=[{'hz': 5.0, 'v_max': 0.6, 'w_max': 1.2}]
        ),
        Node(
            package='cubebot',
            executable='reward_logger',
            name='reward_logger',
            output='screen',
            parameters=[{
                'hz': 10.0,
                'episode_len': 600,
                'min_safe': 0.22,
                'episodes_csv': 'rl_runs/episodes.csv',
                'steps_csv': 'rl_runs/steps.csv'
            }]
        ),
    ])
