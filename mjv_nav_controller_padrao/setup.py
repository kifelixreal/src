from setuptools import setup, find_packages
from os import path

package_name = 'mjv_nav_controller_padrao'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/nav_stack.launch.py']),
        ('share/' + package_name + '/config', ['config/nav2_params.yaml']),
        ('share/' + package_name + '/config', ['config/nav2_params_obstacle.yaml']),
        ('share/' + package_name + '/config', ['config/nav2_params_obstacle_with_zed.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tatiana',
    maintainer_email='you@example.com',
    description='Orchestrator + Arbiter + Controller + VFH with a Nav2 bringup launch.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'orchestrator_execute_path = mjv_nav_controller_padrao.orchestrator_execute_path:main',
            'arbiter_local = mjv_nav_controller_padrao.arbiter_local:main',
            'goal_node = mjv_nav_controller_padrao.goal_node:main',
            'vfh_node = mjv_nav_controller_padrao.vfh_node:main',   # se ainda não tiver, deixe um stub
            'initial_pose_pub = mjv_robot_pkg_with_actions.initial_pose_pub:main',
        ],
    },
)
