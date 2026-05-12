"""
Bringup dos sensores (ZED + RPLidar + TFs estáticas + UI bridge) para um robô.

Multi-robô: passe `namespace:=robotN`. Os frames TF do robô (base, laser)
são prefixados pelo namespace ROS2 (ex: robot2/base_link, robot2/laser).

------------------------------------------------------------------------------
ATENÇÃO — frames TF da ZED (zed-ros2-wrapper)
------------------------------------------------------------------------------
O wrapper da ZED constrói TODOS os seus frames TF a partir do parâmetro
`camera_name`, sem aceitar prefixo de namespace ROS2. Veja:

    mLeftCamFrameId    = mCameraName + "_left_camera_frame";
    mRightCamFrameId   = mCameraName + "_right_camera_frame";
    mImuFrameId        = mCameraName + "_imu_link";
    mCenterFrameId     = mCameraName + "_camera_center";
    ...

(arquivo zed_camera_component_main.cpp, função setTFCoordFrameNames())

Frames TF são strings GLOBAIS no ROS2 — diferente de tópicos, eles NÃO
recebem prefixo de namespace automaticamente. Por isso, se 2 robôs
subirem com camera_name="zed2", ambos publicariam frames com o mesmo
nome ("zed2_left_camera_frame", etc.) e haveria colisão.

Solução adotada: tornamos o `camera_name` dinâmico, prefixando com o
namespace. Para namespace="robot2" e camera_model="zed2", o camera_name
vira "robot2_zed2", produzindo frames únicos por robô:

    robot2_zed2_camera_center
    robot2_zed2_left_camera_frame
    robot2_zed2_left_camera_frame_optical
    robot2_zed2_right_camera_frame
    robot2_zed2_right_camera_frame_optical
    robot2_zed2_imu_link

Também ativamos publish_imu_tf=true para que o ZED publique sozinho a
TF camera_center -> imu_link (com calibração interna correta), em vez
de mantermos um TF estático artificial com offset zero.

NOTA: os frames da ZED usam UNDERSCORE (robot2_zed2_left_*), enquanto
os frames do robô usam BARRA (robot2/base_link). É uma convenção mista,
imposta pela limitação do wrapper. Se algum nó downstream (RViz,
processamento de imagem) precisar referenciar esses frames, usar
exatamente esses nomes.
"""
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
    lidar_serial_port = LaunchConfiguration("lidar_serial_port")

    zed_wrapper_dir = get_package_share_directory("zed_wrapper")

    # Frames TF do robô (prefixados via namespace ROS2 / convenção com barra).
    base_link_frame = [namespace, "/base_link"]
    laser_frame = [namespace, "/laser"]

    # camera_name dinâmico = "<namespace>_<camera_model>" (ex: robot2_zed2).
    # Usamos UNDERSCORE entre namespace e modelo porque o wrapper da ZED
    # concatena _left_camera_frame, _imu_link, etc. ao camera_name.
    camera_name_str = [namespace, "_", camera_model]

    # Frame raiz que o ZED publica internamente: <camera_name>_camera_center
    # Este é o frame ao qual o ZED conecta todos os outros frames internos
    # (left, right, imu, ...) via TFs publicadas pelo próprio nó ZED.
    # IMPORTANTE: precisa ser uma lista PLANA de substitutions (sem listas
    # aninhadas), por isso desempacotamos camera_name_str.
    zed_root_frame = [namespace, "_", camera_model, "_camera_center"]

    # ZED: o launch dela aceita namespace e camera_name nativamente
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_wrapper_dir, "launch", "zed_camera.launch.py")
        ),
        launch_arguments={
            "camera_model": camera_model,
            "namespace": namespace,
            # camera_name dinâmico (gera frames TF únicos por robô).
            "camera_name": camera_name_str,
            "use_container": "false",
            # publish_tf=false: a TF map/odom -> base é responsabilidade
            # do EKF (robot_localization) e do AMCL.
            "publish_tf": "false",
            "publish_map_tf": "false",
            "publish_urdf": "false",
            # publish_imu_tf=true: deixamos o ZED publicar sozinho a TF
            # camera_center -> imu_link (com calibração interna).
            "publish_imu_tf": "true",
        }.items()
    )

    # LiDAR como nó direto
    lidar_node = Node(
        package="sllidar_ros2",
        executable="sllidar_node",
        name="sllidar_node",
        namespace=namespace,
        parameters=[{
            "channel_type": "serial",
            "serial_port": lidar_serial_port,
            "serial_baudrate": 256000,
            "frame_id": laser_frame,
            "inverted": False,
            "angle_compensate": True,
            "scan_mode": "Sensitivity",
        }],
        output="screen",
    )

    # TF estático: <ns>/base_link -> <ns>/laser
    laser_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="laser_tf",
        namespace=namespace,
        arguments=[
            "0.18", "0.0", "0.24", "0", "0", "0", "1",
            base_link_frame, laser_frame,
        ],
        output="screen",
    )

    # TF estático: <ns>/base_link -> <ns>_<modelo>_camera_center
    # Conecta a árvore TF do robô à árvore TF interna da ZED.
    # A partir daqui, o próprio nó ZED publica:
    #   <camera_name>_camera_center -> _left_camera_frame
    #   <camera_name>_camera_center -> _right_camera_frame
    #   <camera_name>_camera_center -> _imu_link
    zed_cam_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="zed_cam_tf",
        namespace=namespace,
        arguments=[
            "0.18", "0.0", "0.25", "0", "0", "0", "1",
            base_link_frame, zed_root_frame,
        ],
        output="screen",
    )

    # Atraso para garantir que a ZED já subiu antes de publicar os TFs estáticos
    tf_static_delayed = TimerAction(
        period=15.0,
        actions=[laser_tf, zed_cam_tf],
    )

    ui_sensors = Node(
        package="mjv_ui_bridge",
        executable="ui_sensors_node",
        name="ui_sensors_node",
        namespace=namespace,
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "camera_model",
            default_value="zed2",
            description="Modelo da câmera ZED (zed2, zed2i, zedm, etc).",
        ),
        DeclareLaunchArgument(
            "namespace",
            default_value="robot2",
            description="Namespace do robô. Prefixa tópicos e frames TF do robô. "
                        "Também é usado como prefixo do camera_name da ZED para "
                        "gerar frames únicos (ex: robot2_zed2_left_camera_frame).",
        ),
        DeclareLaunchArgument(
            "lidar_serial_port",
            default_value="/dev/ttyUSB0",
            description="Porta serial USB do RPLidar.",
        ),
        zed_launch,
        lidar_node,
        tf_static_delayed,
        ui_sensors,
    ])
