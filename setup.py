import os
from glob import glob
from setuptools import setup

package_name = 'zed_camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'scripts'), glob('start_slam_remotely.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marco lapolla',
    maintainer_email='marco.lapolla5@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "camera_node = zed_camera.camera_node:main",
        "tf_camera_node = zed_camera.tf_camera_node:main",
        "slam = zed_camera.slam:main",
        ],
    },
)
