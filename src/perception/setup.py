from setuptools import find_packages, setup

package_name = 'perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
            ['config/apriltag.yaml', 'config/docking.yaml',
             'config/oak_points_filter.yaml']),
        ('share/' + package_name + '/launch',
            ['launch/apriltag.launch.py', 'launch/docking.launch.py',
             'launch/oak_points_filter.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Albin',
    maintainer_email='albin@stenboda.se',
    description='AprilTag detection and docking server bring-up for Modysseus.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'capture_dock_pose = perception.capture_dock_pose:main',
            'image_gate = perception.image_gate:main',
            'tag_dock_server =perception.tag_dock_server:main',
        ],
    },
)
