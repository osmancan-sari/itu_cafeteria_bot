from setuptools import setup
from glob import glob

package_name = 'apriltag_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='',
    maintainer_email='onarane21@itu.edu.tr',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
    'console_scripts': [
        'detect_apriltag = apriltag_detection.detect_apriltag:main',
        'detect_apriltag_3d = apriltag_detection.detect_apriltag_3d:main',
        'follow_apriltag = apriltag_detection.follow_apriltag:main',
    ],
},
)
