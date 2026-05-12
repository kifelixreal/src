from setuptools import setup
from glob import glob
import os

package_name = 'mjv_robot_fleet_navigation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MJV',
    maintainer_email='voce@mjv.com',
    description='Camada de navegação da frota MJV.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'nome_do_executavel = pacote.arquivo:funcao_main'
            'mission_orchestrator = mjv_robot_fleet_navigation.mission_orchestrator:main',
            'teleop_relay = mjv_robot_fleet_navigation.teleop_relay:main',
        ],
    },
)