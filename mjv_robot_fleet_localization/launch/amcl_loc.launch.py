from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params = "/home/mjv/ros2_ws/src/mjv_robot_fleet_localization/config/amcl_nav2.yaml"
    return LaunchDescription([
        Node(package="nav2_map_server", executable="map_server",
             name="map_server", output="screen", parameters=[params]),
        Node(package="nav2_amcl", executable="amcl",
             name="amcl", output="screen", parameters=[params]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
             name="lifecycle_manager_localization", output="screen",
             parameters=[{"autostart": True, "node_names": ["map_server", "amcl"]}]),
    ])
