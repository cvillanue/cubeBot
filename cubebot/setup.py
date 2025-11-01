from setuptools import setup, find_packages
from pathlib import Path

package_name = 'cubebot'

# Fail early if any required file is missing (helps catch typos/paths)
required_paths = [
    f'resource/{package_name}',
    'package.xml',
    'launch/cubebot_sim.launch.py',
    'launch/random_with_logging.launch.py',
    'urdf/cubecar.urdf.xacro',
    'worlds/flat.world',
]
for p in required_paths:
    if not Path(p).exists():
        raise FileNotFoundError(f"setup.py expected file missing: {p}")

setup(
    name=package_name,
    version='0.0.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', [
            'launch/cubebot_sim.launch.py',
            'launch/random_with_logging.launch.py', 
        ]),
        (f'share/{package_name}/urdf', ['urdf/cubecar.urdf.xacro']),
        (f'share/{package_name}/worlds', ['worlds/flat.world']),
        # Add config files explicitly here if/when you have them:
        # (f'share/{package_name}/config', ['config/your.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Callyn Villanueva',
    maintainer_email='cvillanue25@gmail.com',
    description='CubeBot for Gazebo + ROS 2 with basic autonomy and RL scaffolding.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'obstacle_avoider = cubebot.obstacle_avoider:main',
            'random_policy   = cubebot.random_policy:main',
            'scan_logger     = cubebot.scan_logger:main',
            'reward_logger   = cubebot.reward_logger:main',
            'rl_lite         = cubebot.rl_loop:main',
        ],
    },
)
