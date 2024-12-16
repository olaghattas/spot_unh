from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'spot_unh'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'devices'), glob('devices/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='olagh',
    maintainer_email='olaghattas@hotmail.com',
    description='Scripts to use with Spot robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "walk_forward = spot_unh.walk_forward:main",
            "endeff = spot_unh.simple_endeff_movement:main",
            "endeff_odom = spot_unh.endeff_movement_wrt_odom:main",
            "teleop_sm = spot_unh.teleop_spacemouse:main",
            "teleop_sm_body = spot_unh.teleop_spacemouse_body_follows:main",
            "minimal = spot_unh.minimal:main",
        ],
    },
)
