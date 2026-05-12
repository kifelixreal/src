"""
AMCL + map_server + lifecycle_manager para um robô.

Decisão de design: o map_server fica FORA do namespace dos robôs porque o
mapa é compartilhado entre toda a frota (mesmo ambiente físico). O AMCL
roda dentro do namespace de cada robô.

Multi-robô: passe `namespace:=robotN`. Para subir um único map_server
compartilhado, deixe `share_map:=true` apenas no PRIMEIRO robô que subir
(os demais usam `share_map:=false`).

Estratégia de substituição de frames:
  Como nem todas as instalações têm `nav2_common.launch.RewrittenYaml`
  disponível, usamos text-substitution manual: o YAML pode conter o
  token `<NAMESPACE>` que é substituído pelo namespace real antes do
  carregamento.
"""
import os
import tempfile

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _materialize_amcl_yaml(namespace: str, src_yaml: str) -> str:
    """Lê o YAML, substitui <NAMESPACE> e injeta os frames com prefixo.

    Como o YAML do AMCL não tem placeholders <NAMESPACE> (frames são
    deixados sem prefixo), fazemos o trabalho aqui: lemos o YAML,
    sobrescrevemos as chaves de frame e gravamos um arquivo temp.
    """
    import yaml as pyyaml

    with open(src_yaml, "r", encoding="utf-8") as f:
        data = pyyaml.safe_load(f)

    base_frame = f"{namespace}/base_link"
    odom_frame = f"{namespace}/odom"
    scan_topic = "scan"  # já vira robotN/scan via namespace do nó

    # Sobrescreve frames do AMCL. Aceita YAML com chave wildcard /** ou
    # com nome direto amcl:.
    def _patch_amcl(node_section):
        params = node_section.get("ros__parameters", {})
        params["odom_frame_id"] = odom_frame
        params["base_frame_id"] = base_frame
        params["global_frame_id"] = "map"
        params["scan_topic"] = scan_topic
        node_section["ros__parameters"] = params

    if "/**" in data:
        wild = data["/**"]
        if "amcl" in wild:
            _patch_amcl(wild["amcl"])
    if "amcl" in data:
        _patch_amcl(data["amcl"])

    tmp = tempfile.NamedTemporaryFile(
        mode="w",
        prefix=f"amcl_{namespace}_",
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
    params_file = LaunchConfiguration("params_file").perform(context)
    share_map = LaunchConfiguration("share_map").perform(context)

    materialized = _materialize_amcl_yaml(namespace, params_file)

    nodes = []

    if share_map.lower() == "true":
        # map_server compartilhado (fora do namespace)
        nodes.append(Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[params_file],  # pega yaml_filename do YAML original
        ))
        nodes.append(Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_map",
            output="screen",
            parameters=[{
                "autostart": True,
                "node_names": ["map_server"],
            }],
        ))

    # AMCL dentro do namespace
    nodes.append(Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        namespace=namespace,
        output="screen",
        parameters=[materialized],
    ))

    # Lifecycle manager do AMCL (dentro do namespace)
    nodes.append(Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        namespace=namespace,
        output="screen",
        parameters=[{
            "autostart": True,
            "node_names": ["amcl"],
        }],
    ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "namespace",
            default_value="robot2",
            description="Namespace do robô.",
        ),
        DeclareLaunchArgument(
            "params_file",
            description="YAML com parâmetros AMCL e map_server.",
        ),
        DeclareLaunchArgument(
            "share_map",
            default_value="true",
            description="Se true, sobe o map_server compartilhado (fora de namespace). "
                        "Para múltiplos robôs no mesmo ambiente, deixe true só no primeiro.",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
