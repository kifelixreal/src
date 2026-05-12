from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    camera_model = LaunchConfiguration("camera_model")
    namespace = LaunchConfiguration("namespace")

    zed_wrapper_dir = get_package_share_directory("zed_wrapper")

    # ZED: o próprio launch dela já aceita namespace
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_wrapper_dir, "launch", "zed_camera.launch.py")
        ),
        launch_arguments={
            "camera_model": camera_model,
            #"namespace": namespace,
            #"camera_name": "zed",
            "use_container": "false",
            "publish_tf": "false",
            "publish_map_tf": "false",
            #"publish_urdf": "false",
            #"publish_imu_tf": "false",
        }.items()
    )

    # LiDAR direto como node, sem depender do launch pronto
    lidar_node = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        namespace=namespace,
        parameters=[{
            "channel_type": "serial",
            "serial_port": "/dev/ttyUSB0",
            "serial_baudrate": 256000,
            "frame_id": "laser",
            "inverted": False,
            "angle_compensate": True,
            "scan_mode": "Sensitivity",
        }],
        output="screen",
    )

    # TF estático do LIDAR
    laser_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="laser_tf",
        namespace=namespace,
        arguments=["0.18", "0.0", "0.24", "0", "0", "0", "1", "base_link", "laser"],
        output="screen",
    )

    # TFs estáticas da ZED
    zed_cam_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="zed_cam_tf",
        namespace=namespace,
        arguments=["0.18", "0.0", "0.25", "0", "0", "0", "1", "base_link", "zed_camera_link"],
        output="screen",
    )

    zed_imu_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="zed_imu_tf",
        namespace=namespace,
        arguments=["0.0", "0.0", "0.0", "0", "0", "0", "1", "zed_camera_link", "zed_imu_link"],
        output="screen",
    )

    tf_static_delayed = TimerAction(
        period=15.0,
        actions=[laser_tf, zed_cam_tf, zed_imu_tf],
    )

    ui_sensors = Node(
        package="mjv_ui_bridge",
        executable="ui_sensors_node",
        name="ui_sensors_node",
        namespace=namespace,
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("camera_model", default_value="zed2i"),
        DeclareLaunchArgument("namespace", default_value=""),
        zed_launch,
        lidar_node,
        tf_static_delayed,
        ui_sensors,
    ])