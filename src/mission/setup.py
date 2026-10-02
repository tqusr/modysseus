from setuptools import find_packages, setup

package_name = 'mission'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/docks.yaml']),
        ('share/' + package_name + '/launch',
            ['launch/mission.launch.py', 'launch/full_stack.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Albin',
    maintainer_email='albin@stenboda.se',
    description='Mission behavior tree, Saga AI client, and stub Nav2/docking actions for Modysseus.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_node = mission.mission_node:main',
            'stub_action_servers = mission.stub_action_servers:main',
        ],
    },
)
