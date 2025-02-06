from setuptools import find_packages, setup

import os

from glob import glob


package_name = 'hardware_package'


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

        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),

        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.xacro'))),

    ],

    install_requires=['setuptools'],

    zip_safe=True,

    maintainer='Andy West',

    maintainer_email='andrew.west@manchester.ac.uk',

    description='TODO: Package description',

    license='MIT',

    tests_require=['pytest'],

    entry_points={

        'console_scripts': [
            'lidar_node = hardware_package.lidar_node:main'

        ],

    },

)
