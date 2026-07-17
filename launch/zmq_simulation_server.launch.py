from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = get_package_share_directory("network_bridge")
    camera_config = config_dir + "/config/ZmqCameraServerSimulation.yaml"
    state_config = config_dir + "/config/ZmqStateServer.yaml"

    return LaunchDescription(
        [
            Node(
                package="network_bridge",
                executable="network_bridge",
                name="zmq_bridge_server",
                output="screen",
                parameters=[camera_config],
            ),
            Node(
                package="network_bridge",
                executable="network_bridge",
                name="zmq_bridge_server_state",
                output="screen",
                parameters=[state_config],
            ),
        ]
    )
