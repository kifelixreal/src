#!/usr/bin/env python3
"""
ROS2 Node: Reactive Obstacle Avoidance (frame: map)
Autor: Johann Amorim (adaptado)
Descrição:
  - Lê LaserScan
  - Detecta obstáculos
  - Gera subgoal no frame 'map'
  - Publica PoseStamped global para navegação
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
import numpy as np
import math
import tf_transformations

class ReactiveAvoidance(Node):
    def __init__(self):
        super().__init__('reactive_avoidance_node')

        # --- Subscribers ---
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # --- Publishers ---
        self.goal_pub = self.create_publisher(PoseStamped, '/subgoal', 10)

        # --- TF Listener ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Parâmetros configuráveis ---
        self.safe_distance = 0.6      # m - distância mínima segura
        self.max_range = 2.0          # m - distância máxima de análise
        self.goal_distance = 1.0      # m - distância até o subgoal

        self.get_logger().info("✅ Reactive Obstacle Avoidance Node iniciado (frame: map).")

    def scan_callback(self, msg: LaserScan):
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # Filtrar leituras inválidas
        valid = np.isfinite(ranges)
        ranges = np.clip(ranges[valid], 0.0, self.max_range)
        angles = angles[valid]

        if len(ranges) == 0:
            self.get_logger().warn("⚠️ Nenhuma leitura válida do LiDAR.")
            return

        # Direções livres
        free_mask = ranges > self.safe_distance
        if not np.any(free_mask):
            self.get_logger().warn("⚠️ Nenhum caminho livre detectado.")
            return

        # Seleciona ângulo com maior distância (direção livre)
        idx_best = np.argmax(ranges)
        best_angle = angles[idx_best]
        best_dist = ranges[idx_best]

        # Coordenadas locais (frame base_link)
        goal_local_x = self.goal_distance * math.cos(best_angle)
        goal_local_y = self.goal_distance * math.sin(best_angle)

        # --- Transformar para o frame map ---
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time())

            # Pose atual do robô em map
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            q = transform.transform.rotation
            _, _, yaw = tf_transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

            # Transformar o ponto do frame base_link para map
            goal_map_x = tx + (goal_local_x * math.cos(yaw) - goal_local_y * math.sin(yaw))
            goal_map_y = ty + (goal_local_x * math.sin(yaw) + goal_local_y * math.cos(yaw))

            self.publish_subgoal(goal_map_x, goal_map_y)

            self.get_logger().info(
                f"➡️ Subgoal (map): ({goal_map_x:.2f}, {goal_map_y:.2f}) | Ângulo livre = {math.degrees(best_angle):.1f}°"
            )

        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn("⚠️ Falha ao obter TF map←base_link. Usando coordenadas locais.")
            self.publish_subgoal(goal_local_x, goal_local_y, frame='base_link')

    def publish_subgoal(self, x, y, frame='map'):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ReactiveAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Encerrando nó...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
