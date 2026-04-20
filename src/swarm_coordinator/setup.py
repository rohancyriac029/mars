from setuptools import setup

package_name = 'swarm_coordinator'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/multi_robot_swarm.launch.py']),
        (
            'share/' + package_name + '/config',
            [
                'config/nav2_params_robot1.yaml',
                'config/nav2_params_robot2.yaml',
                'config/nav2_params_robot3.yaml',
                'config/nav2_params_robot4.yaml',
            ],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='swarm_user',
    maintainer_email='you@example.com',
    description='Minimal multi-robot swarm goal coordinator using Nav2 actions.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'goal_coordinator = swarm_coordinator.goal_coordinator:main',
            'pose_initializer = swarm_coordinator.pose_initializer:main',
        ],
    },
)
