from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'waiter_robot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='erenonaran',
    maintainer_email='onarane21@itu.edu.tr',
    description='Robot description package for waiter robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            f'state_publisher = waiter_robot_description.state_publisher:main'
        ],
    },
)
