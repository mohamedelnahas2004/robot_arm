from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'arm_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zed',
    maintainer_email='eyadmezed@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dynamic_pnp = arm_control.dynamic_pnp:main',
            'vision_node = arm_control.vision_node:main',
            'tracker_node = arm_control.tracker_node:main',
            'translator_node = arm_control.translator_node:main',
            'teleop_init    = arm_control.teleop_init_node:main'
        ],
    },
)
