import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Define LaunchDescription variable
    ld = LaunchDescription()

    camera_model = 'zed2'

    # Camera name
    camera_name = 'zed2'


    # ZED Configurations to be loaded by ZED Node
    config_common = os.path.join(
        get_package_share_directory('zed_camera'),
        'config',
        'common.yaml'
    )

    config_camera = os.path.join(
        get_package_share_directory('zed_camera'),
        'config',
        camera_model + '.yaml'
    )

    # Set LOG format
    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time} [{name}] [{severity}] {message}'

    # ZED Wrapper node
    zed_wrapper_node = Node(
        package='zed_wrapper',
        node_namespace="/"+camera_name,
        node_executable='zed_wrapper',
        node_name='zed_node',
        output='screen',
        parameters=[
            config_common,  # Common parameters
            config_camera,  # Camera related parameters
        ],
        remappings=[
                    ("/zed2/zed_node/imu/data_raw", "/zed_camera/imu"),
                    ("/zed2/zed_node/left_raw/image_raw_gray", "/zed_camera/raw_frame"),
                    ("/zed2/zed_node/odom", "/zed_camera/odom"),
                    ("/zed2/zed_node/point_cloud/cloud_registered", "/zed_camera/point_cloud"),
                   ]
    )

    # Add nodes to LaunchDescription
    ld.add_action(rsp_node)
    ld.add_action(zed_wrapper_node)

    return ld
