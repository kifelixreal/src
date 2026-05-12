"""
Camada Nav2 + nós MJV (mission_orchestrator + teleop_relay) para um robô.

Multi-robô: passe `namespace:=robotN`. O nav2_bringup é instanciado dentro
do namespace, e os parâmetros são reescritos para usar frames TF
prefixados (robotN/base_link, robotN/odom, robotN/scan).

Estratégia de substituição de frames:
  - Os frames TF que precisam de prefixo são marcados no YAML como
    `<NAMESPACE>/foo` (ex: `<NAMESPACE>/odom`).
  - Antes do nav2_bringup carregar o YAML, geramos um arquivo temporário
    em /tmp com o namespace real substituído via PyYAML. Adicionalmente,
    sobrescrevemos `robot_base_frame` e o tópico `scan` em todos os nós.
  - Não dependemos de `nav2_common.launch.RewrittenYaml` para evitar
    problemas de instalação.
"""
import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _materialize_nav2_yaml(namespace: str, src_yaml: str) -> str:
    """Substitui placeholder <NAMESPACE> e injeta robot_base_frame/scan topic.

    Retorna o caminho de um arquivo YAML temporário pronto para o Nav2.
    """
    import yaml as pyyaml

    # 1) Text substitution do <NAMESPACE> em texto puro (cobre frames que
    #    foram marcados como <NAMESPACE>/foo no YAML original).
    with open(src_yaml, "r", encoding="utf-8") as f:
        raw = f.read()
    rewritten_text = raw.replace("<NAMESPACE>", namespace)

    # 2) Reparse e injeta frames + topics dependentes do namespace.
    data = pyyaml.safe_load(rewritten_text)

    base_frame = f"{namespace}/base_link"
    scan_topic = f"{namespace}/scan"

    def _walk(obj):
        if isinstance(obj, dict):
            for k, v in list(obj.items()):
                if k == "robot_base_frame":
                    obj[k] = base_frame
                elif k == "topic" and isinstance(v, str) and v in ("scan", "/scan"):
                    obj[k] = scan_topic
                else:
                    _walk(v)
        elif isinstance(obj, list):
            for item in obj:
                _walk(item)

    _walk(data)

    tmp = tempfile.NamedTemporaryFile(
        mode="w",
        prefix=f"nav2_{namespace}_",
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
    src_yaml = LaunchConfiguration("params_file").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    missions_file = LaunchConfiguration("missions_file").perform(context)

    materialized_yaml = _materialize_nav2_yaml(namespace, src_yaml)
    base_frame = f"{namespace}/base_link"

    nav2_dir = get_package_share_directory("nav2_bringup")

    # Nav2 dentro do namespace
    nav2_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": materialized_yaml,
            "namespace": namespace,
        }.items()
    )

    # Mission Orchestrator
    mission_orchestrator_node = Node(
        package="mjv_robot_fleet_navigation_teste",
        executable="mission_orchestrator",
        name="mission_orchestrator",
        namespace=namespace,
        output="screen",
        parameters=[{
            "use_sim_time": (use_sim_time.lower() == "true"),
            "plan_topic": "plan",
            "navigate_action": "navigate_to_pose",
            "global_frame": "map",
            "base_link_frame": base_frame,
            "missions_file": missions_file,
            "reach_tol": 0.3,
            "progress_hz": 5.0,
        }],
    )

    # Teleop Relay
    teleop_relay_node = Node(
        package="mjv_robot_fleet_navigation_teste",
        executable="teleop_relay",
        name="teleop_relay",
        namespace=namespace,
        output="screen",
        parameters=[{"watchdog_timeout": 0.5}],
    )

    return [nav2_include, mission_orchestrator_node, teleop_relay_node]


def generate_launch_description():
    pkg_dir = get_package_share_directory("mjv_robot_fleet_navigation_teste")
    default_params = os.path.join(pkg_dir, "config", "nav2_params.yaml")
    default_missions = os.path.join(pkg_dir, "config", "missions.json")

    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace",
            default_value="robot2",
            description="Namespace do robô.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use clock de simulação.",
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=default_params,
            description="YAML do Nav2 (com placeholder <NAMESPACE> nos frames).",
        ),
        DeclareLaunchArgument(
            "missions_file",
            default_value=default_missions,
            description="JSON com as missões do mission_orchestrator.",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
