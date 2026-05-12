from setuptools import setup
from glob import glob
import os

package_name = 'mjv_robot_fleet_sensors_teste'

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
    description='Launches de sensores da frota MJV.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={},
)