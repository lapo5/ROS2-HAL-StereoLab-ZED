import os

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define LaunchDescription variable
    ld = LaunchDescription()

    camera_model = 'zed2'

    # ZED Configurations to be loaded by ZED Node
    config_common = os.path.join(
        get_package_share_directory('zed_camera'),
        'params',
        'common_only_localisation.yaml'
    )

    config_camera = os.path.join(
        get_package_share_directory('zed_camera'),
        'params',
        camera_model + '.yaml'
    )

    # Set LOG format
    os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time} [{name}] [{severity}] {message}'

    # ZED Wrapper node
    zed_wrapper_node = Node(
        package='zed_wrapper',
        node_executable='zed_wrapper',
        node_name='zed_node',
        output='screen',
        parameters=[
            config_common,  # Common parameters
            config_camera,  # Camera related parameters
        ],
        remappings=[
                    ("/zed_node/odom", "/zed_camera/odom"),
        ]
    )

    # Add nodes to LaunchDescription
    ld.add_action(zed_wrapper_node)

    return ld
