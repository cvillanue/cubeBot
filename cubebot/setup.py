from setuptools import setup, find_packages

package_name = 'cubebot'

setup(
    name=package_name,
    version='0.0.2',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/cubebot_sim.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/cubecar.urdf.xacro']),
        ('share/' + package_name + '/worlds', ['worlds/flat.world']),
        ('share/' + package_name + '/config', []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Callyn Villanueva',
    maintainer_email='cvillanue25@gmail.com',
    description='Cube Bot for Gazebo + ROS 2 with basic autonomy and RL scaffolding.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'obstacle_avoider = cubebot.obstacle_avoider:main',
            'random_policy = cubebot.random_policy:main',
            'scan_logger = cubebot.scan_logger:main',
        ],
    },
)
