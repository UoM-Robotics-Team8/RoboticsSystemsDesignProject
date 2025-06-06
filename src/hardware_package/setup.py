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

        # Include URDF (.urdf) files
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf')) + glob(os.path.join('urdf', '*.xacro'))),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.xacro'))),
	    # Include mesh (.stl) files
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.stl'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.dae'))),

        # Include world (.sdf or dae) files
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*/*.[sd][da][fe]'), recursive=True)),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.sdf'))),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include config (.yaml) files
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*yaml*'))),
	    # Include map (.yaml and .pgm) files
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*.[yp][ag][m]'))),
        # Include rviz (.rviz) files
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
        # Include behaviour tree xmls
        (os.path.join('share', package_name, 'behaviour'), glob(os.path.join('behaviour', '*.xml'))),    ],

    install_requires=['setuptools'],

    zip_safe=True,

    maintainer='Andy West',

    maintainer_email='andrew.west@manchester.ac.uk',

    description='TODO: Package description',

    license='MIT',

    tests_require=['pytest'],

    entry_points={

        'console_scripts': [
            'lidar_node = hardware_package.lidar_node:main',
            'periodic_map_saver = hardware_package.periodic_map_saver:main',
            'object_detection_listener = hardware_package.object_detection_listener:main',
            'pointcloud_to_laserscan = hardware_package.pointcloud_to_laserscan:main',
        ],

    },

)
