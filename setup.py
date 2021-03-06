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
        # Include all launch files. This is the most important line here!
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marco lapolla',
    maintainer_email='marco.lapolla5@gmail.com',
    description='HAL for ZED2 Camera',
    license='BSD',
    entry_points={
        'console_scripts': [
        "camera_node = zed_camera.camera_node:main",
        "localisation = zed_camera.localisation:main",
        "slam = zed_camera.slam:main",
        ],
    },
)
