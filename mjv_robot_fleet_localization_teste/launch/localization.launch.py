"""
Camada completa de localização para um robô:
  - EKF (robot_localization) fundindo wheel/odom + zed/odom + zed/imu
  - AMCL + map_server (via amcl_loc.launch.py)
  - localization_manager (rotina de boot que tenta checkpoint, faz spin/fb
    e estabiliza o AMCL)

Multi-robô: passe `namespace:=robotN`. Para múltiplos robôs no mesmo
ambiente, deixe `share_map:=true` apenas no primeiro robô a subir.

Estratégia de substituição de frames: usamos text-substitution manual
(via PyYAML) para não depender de `nav2_common.launch.RewrittenYaml`,
que pode não estar instalado em todos os ambientes.
"""
import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _materialize_ekf_yaml(namespace: str, src_yaml: str) -> str:
    """Lê o YAML do EKF e injeta os frames com prefixo de namespace."""
    import yaml as pyyaml

    with open(src_yaml, "r", encoding="utf-8") as f:
        data = pyyaml.safe_load(f)

    base_frame = f"{namespace}/base_link"
    odom_frame = f"{namespace}/odom"

    def _patch(node_section):
        params = node_section.get("ros__parameters", {})
        params["world_frame"] = odom_frame
        params["odom_frame"] = odom_frame
        params["base_link_frame"] = base_frame
        node_section["ros__parameters"] = params

    if "/**" in data:
        wild = data["/**"]
        if "ekf_odom" in wild:
            _patch(wild["ekf_odom"])
        elif "ros__parameters" in wild:
            # caso o YAML use /** direto sem nó intermediário
            _patch(wild)
    if "ekf_odom" in data:
        _patch(data["ekf_odom"])

    tmp = tempfile.NamedTemporaryFile(
        mode="w",
        prefix=f"ekf_{namespace}_",
        suffix=".yaml",
        delete=False,
        encoding="utf-8",
    )
    pyyaml.safe_dump(data, tmp)
    tmp.flush()
    tmp.close()
    return tmp.name


def _launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace").perform(context)
    ekf_yaml = LaunchConfiguration("ekf_yaml").perform(context)
    amcl_yaml = LaunchConfiguration("amcl_yaml").perform(context)
    last_pose_path = LaunchConfiguration("last_pose_path").perform(context)
    share_map = LaunchConfiguration("share_map").perform(context)

    materialized_ekf = _materialize_ekf_yaml(namespace, ekf_yaml)

    share_dir = get_package_share_directory("mjv_robot_fleet_localization_teste")

    # EKF dentro do namespace
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_odom",
        namespace=namespace,
        output="screen",
        parameters=[materialized_ekf],
    )
    ekf_delayed = TimerAction(period=1.0, actions=[ekf_node])

    # AMCL + map_server via amcl_loc.launch.py
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share_dir, "launch", "amcl_loc.launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "params_file": amcl_yaml,
            "share_map": share_map,
        }.items()
    )
    amcl_delayed = TimerAction(period=5.0, actions=[amcl_launch])

    # Localization manager (rotina de inicialização) dentro do namespace
    init_loc = Node(
        package="mjv_robot_fleet_localization_teste",
        executable="localization_manager",
        name="localization_manager",
        namespace=namespace,
        parameters=[{
            "mode": "boot",
            "last_pose_path": last_pose_path,
            "spin_wz": 0.35,
            "spins": 1,
            "do_fb": True,
        }],
        output="screen",
    )
    # init_loc_delayed = TimerAction(period=10.0, actions=[init_loc])

    return [ekf_delayed, amcl_delayed]  # init_loc fica comentado como antes


def generate_launch_description():
    share_dir = get_package_share_directory("mjv_robot_fleet_localization_teste")

    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace",
            default_value="robot2",
            description="Namespace do robô.",
        ),
        DeclareLaunchArgument(
            "ekf_yaml",
            default_value=os.path.join(share_dir, "config", "ekf.yaml"),
            description="YAML do EKF (robot_localization).",
        ),
        DeclareLaunchArgument(
            "amcl_yaml",
            default_value=os.path.join(share_dir, "config", "amcl_nav2.yaml"),
            description="YAML do AMCL + map_server.",
        ),
        DeclareLaunchArgument(
            "last_pose_path",
            default_value="/home/mjv/.mjv/last_pose.yaml",
            description="Arquivo de checkpoint da última pose conhecida. "
                        "Por robô, recomenda-se sobrescrever para "
                        "/home/mjv/.mjv/<namespace>/last_pose.yaml.",
        ),
        DeclareLaunchArgument(
            "share_map",
            default_value="true",
            description="Se true, sobe o map_server compartilhado. Para múltiplos "
                        "robôs no mesmo ambiente, deixe true só no primeiro.",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
