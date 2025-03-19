from setuptools import find_packages, setup

package_name = 'px100_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy'],
    zip_safe=True,
    maintainer='mscrobotics2425laptop42',
    maintainer_email='alyhatemtawfik@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'px100_node = px100_control.px100_node:main'
        ],
    },
)
