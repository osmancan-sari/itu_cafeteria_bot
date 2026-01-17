from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cafeteria_robot_fsm'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Install marker file for ament
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Install package.xml
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hakan',
    maintainer_email='hakan66@example.com',
    description='High-Level State Machine for ITU Cafeteria Robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Main FSM node - this is how you run it:
            # ros2 run cafeteria_robot_fsm robot_state_machine
            'robot_state_machine = cafeteria_robot_fsm.robot_state_machine:main',
            # Operator control panel - interactive CLI for human operators:
            # ros2 run cafeteria_robot_fsm operator_panel
            'operator_panel = cafeteria_robot_fsm.operator_panel:main',
        ],
    },
)
