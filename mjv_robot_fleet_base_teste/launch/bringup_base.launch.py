"""
Bringup da base (controle ODrive via CAN + odometria) para um robô.

Multi-robô: passe `namespace:=robotN` para cada instância.
Exemplos:
  ros2 launch mjv_robot_fleet_base_teste bringup_base.launch.py namespace:=robot1
  ros2 launch mjv_robot_fleet_base_teste bringup_base.launch.py namespace:=robot2 can_channel:=can1
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("mjv_robot_fleet_base_teste")

    default_params_file = os.path.join(pkg_share, "config", "robot_default.yaml")
    default_dbc_file = os.path.join(pkg_share, "odrive-cansimple.dbc")

    namespace = LaunchConfiguration("namespace")
    params_file = LaunchConfiguration("params_file")
    can_channel = LaunchConfiguration("can_channel")
    dbc_file = LaunchConfiguration("dbc_file")

    # Frames TF prefixados pelo namespace (cada robô tem sua própria árvore TF
    # exceto o frame global "map", que é compartilhado).
    odom_frame = [namespace, "/odom"]
    base_frame = [namespace, "/base_link"]

    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace",
            default_value="robot2",
            description="Namespace do robô (ex: robot1, robot2). Prefixa todos os tópicos e frames TF.",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params_file,
            description="Caminho do YAML com parâmetros do nó de odometria.",
        ),
        DeclareLaunchArgument(
            "can_channel",
            default_value="can0",
            description="Interface CAN (socketcan) usada pelo ODrive.",
        ),
        DeclareLaunchArgument(
            "dbc_file",
            default_value=default_dbc_file,
            description="Caminho do arquivo DBC do ODrive CANSimple.",
        ),
        Node(
            package="mjv_robot_fleet_base_teste",
            executable="odom_node_copy",
            name="odom_node",
            namespace=namespace,
            output="screen",
            parameters=[
                params_file,
                {
                    # Overrides dependentes de namespace — precisam ser
                    # injetados aqui porque o YAML não conhece o namespace
                    # em tempo de parse.
                    "odom_frame": odom_frame,
                    "base_frame": base_frame,
                    "can_channel": can_channel,
                    "can_dbc": dbc_file,
                }
            ],
        ),
    ])
