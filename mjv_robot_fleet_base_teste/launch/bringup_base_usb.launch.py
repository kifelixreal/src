"""
Bringup da base via USB (alternativa ao CAN) + odometria.

Multi-robô: passe `namespace:=robotN` para cada instância.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("mjv_robot_fleet_base_teste")
    default_params = os.path.join(pkg_share, "config", "robot_default.yaml")

    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")

    odom_frame = [namespace, "/odom"]
    base_frame = [namespace, "/base_link"]

    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace",
            default_value="robot2",
            description="Namespace do robô. Prefixa tópicos e frames TF.",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="YAML com parâmetros do nó.",
        ),
        Node(
            package="mjv_robot_fleet_base_teste",
            executable="odom_node_usb",
            name="odom_node_usb",
            namespace=namespace,
            output="screen",
            parameters=[
                params_file,
                {
                    "odom_frame": odom_frame,
                    "base_frame": base_frame,
                },
            ],
        ),
    ])
