#!/usr/bin/env python3
import math, time
from typing import List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformListener

from mjv_robot_interfaces.action import ExecutePath

# ---------------- utils ----------------
def quat_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion(); q.z = math.sin(yaw/2.0); q.w = math.cos(yaw/2.0); return q

def dist_xy(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(b[0]-a[0], b[1]-a[1])

def path_to_xy_list(path: Path) -> List[Tuple[float, float]]:
    return [(ps.pose.position.x, ps.pose.position.y) for ps in path.poses]

def cumulative_length(points: List[Tuple[float, float]]) -> float:
    if len(points) <= 1: return 0.0
    return sum(dist_xy(points[i-1], points[i]) for i in range(1, len(points)))

def nearest_index(points: List[Tuple[float, float]], p: Tuple[float, float]) -> int:
    if not points: return 0
    best_i, best_d = 0, 1e18
    for i, xy in enumerate(points):
        d = dist_xy(xy, p)
        if d < best_d: best_i, best_d = i, d
    return best_i

def remaining_distance(points: List[Tuple[float, float]], k: int, p: Tuple[float, float]) -> float:
    if not points: return 0.0
    k = max(0, min(k, len(points)-1))
    d = dist_xy(p, points[k])
    for i in range(k+1, len(points)):
        d += dist_xy(points[i-1], points[i])
    return d


# --------------- Orchestrator que fala com o arbiter_local ---------------
class ExecutePathOrchestrator(Node):
    """
    Faz:
      • Recebe ExecutePath (/execute_path), publica /path_resampled (latched) para o arbiter_local
      • Publica um far-goal em /goal (opcional, se você usa no VFH)
      • Encaminha comandos de controle PAUSE/RESUME/CANCEL para o arbiter via /path_ctrl
      • Monitora TF para gerar feedback e detectar chegada

    NÃO faz:
      × Publicar cmd_vel (freio é do arbiter_local)
      × Escolher subgoal (isso é do arbiter_local)
    """

    def __init__(self):
        super().__init__('orchestrator_execute_path')

        # ---- parâmetros ----
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('base_link_frame', 'base_link')
        self.declare_parameter('path_topic', '/path')     # arbiter lê
        self.declare_parameter('far_goal_topic', '/goal')           # se seu VFH usa um alvo distante
        self.declare_parameter('ctrl_in_topic', '/mission_ctrl')    # vem do MissionOrchestrator (UI)
        self.declare_parameter('ctrl_out_topic', '/path_ctrl')      # vai para o arbiter_local

        self.declare_parameter('reach_tol', 0.20)
        self.declare_parameter('feedback_hz', 5.0)
        self.declare_parameter('lookahead_idx_cap', 3)

        # ---- lê params ----
        self.global_frame      = self.get_parameter('global_frame').value
        self.base_link_frame   = self.get_parameter('base_link_frame').value
        self.path_topic        = self.get_parameter('path_topic').value
        self.far_goal_topic    = self.get_parameter('far_goal_topic').value
        self.ctrl_in_topic     = self.get_parameter('ctrl_in_topic').value
        self.ctrl_out_topic    = self.get_parameter('ctrl_out_topic').value
        self.reach_tol         = float(self.get_parameter('reach_tol').value)
        self.feedback_hz       = float(self.get_parameter('feedback_hz').value)
        self.lookahead_idx_cap = int(self.get_parameter('lookahead_idx_cap').value)

        # ---- TF ----
        self.tf = Buffer(cache_time=Duration(seconds=10.0))
        self.tl = TransformListener(self.tf, self)

        # ---- publishers ----
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        self.pub_path = self.create_publisher(Path, self.path_topic, latched_qos)
        self.pub_far  = self.create_publisher(PoseStamped, self.far_goal_topic, 10)
        self.pub_ctrl = self.create_publisher(String, self.ctrl_out_topic, 10)  # -> arbiter_local

        # ---- sub de controle (opcional) ----
        # se você já publica /path_ctrl direto do MissionOrchestrator, pode remover essa assinatura
        self.create_subscription(String, self.ctrl_in_topic, self._on_ctrl_in, 10)

        # ---- estado ----
        self._active = False
        self._paused = False
        self._points: List[Tuple[float, float]] = []
        self._total_points = 0
        self._last_idx = 0
        self._gh = None

        # ---- ActionServer ----
        self._as = ActionServer(
            self, ExecutePath, '/execute_path',
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            execute_callback=self._on_execute
        )

        # ---- feedback timer ----
        self._fb_timer = self.create_timer(
            1.0 / max(1e-3, self.feedback_hz), self._on_feedback_tick
        )

        self.get_logger().info(
            f"[orchestrator] up | action=/execute_path | path->{self.path_topic} | ctrl_out->{self.ctrl_out_topic}"
        )

    # ---------- controle (repasse para arbiter) ----------
    def _send_ctrl(self, cmd: str):
        msg = String(); msg.data = cmd
        self.pub_ctrl.publish(msg)
        self.get_logger().info(f"[orchestrator] ctrl->{self.ctrl_out_topic}: {cmd}")

    def _on_ctrl_in(self, msg: String):
        cmd = (msg.data or "").strip().upper()
        if cmd == "PAUSE":
            self._paused = True
            self._send_ctrl("PAUSE")
        elif cmd == "RESUME":
            self._paused = False
            self._send_ctrl("RESUME")
        elif cmd == "CANCEL":
            # não encerra a Action imediatamente; somente sinaliza para o arbiter
            self._send_ctrl("CANCEL")
            # opcional: também marcar inativo aqui
            # self._active = False
        else:
            self.get_logger().warn(f"[orchestrator] comando desconhecido em {self.ctrl_in_topic}: '{cmd}'")

    # ---------- Action handlers ----------
    def _on_goal(self, goal_req: ExecutePath.Goal):
        if self._active:
            self.get_logger().warn("Novo goal: preempção do atual.")
        self._paused = False
        # ao aceitar novo path, já manda RESUME ao arbiter
        self._send_ctrl("RESUME")
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle):
        self.get_logger().info("Cancel solicitado.")
        self._active = False
        self._send_ctrl("CANCEL")  # arbiter vai publicar halt/segurar controller
        return CancelResponse.ACCEPT

    def _on_execute(self, goal_handle):
        self._gh = goal_handle
        path_msg: Path = goal_handle.request.path

        if not path_msg.poses:
            self.get_logger().error("Path vazio — abortando.")
            goal_handle.abort(); self._gh = None
            return ExecutePath.Result(success=False, message="Path vazio")

        # carimbo e frame
        now = self.get_clock().now().to_msg()
        path_msg.header.stamp = now
        if not path_msg.header.frame_id:
            path_msg.header.frame_id = self.global_frame
        for ps in path_msg.poses:
            ps.header.stamp = now
            if not ps.header.frame_id:
                ps.header.frame_id = path_msg.header.frame_id

        # publica path latched (arbiter lê)
        self.pub_path.publish(path_msg)

        # publica far-goal (se usado pelo VFH)
        last = path_msg.poses[-1]
        far = PoseStamped()
        far.header = Header(frame_id=path_msg.header.frame_id, stamp=now)
        far.pose.position.x = last.pose.position.x
        far.pose.position.y = last.pose.position.y
        far.pose.orientation = last.pose.orientation if (last.pose.orientation.w or last.pose.orientation.z) else quat_from_yaw(0.0)
        self.pub_far.publish(far)

        # estado
        self._points = path_to_xy_list(path_msg)
        self._total_points = len(self._points)
        self._last_idx = 0
        self._active = True
        self._paused = False

        self.get_logger().info(
            f"[orchestrator] path publicado: {self._total_points} pts, L≈{cumulative_length(self._points):.2f} m"
        )

        # loop leve (arbiter toca o baixo nível)
        while rclpy.ok() and self._active:
            if goal_handle.is_cancel_requested:
                self._active = False
                self._send_ctrl("CANCEL")
                goal_handle.canceled(); self._gh = None
                return ExecutePath.Result(success=False, message="Cancelado")
            # em pausa, só mantenha vivo; quem para o robô é o arbiter
            if self._paused:
                time.sleep(0.05)
                continue
            time.sleep(0.05)

        # terminou (chegada detectada no feedback tick ou cancel)
        if goal_handle.is_cancel_requested:
            goal_handle.canceled(); self._gh = None
            return ExecutePath.Result(success=False, message="Cancelado")

        goal_handle.succeed(); self._gh = None
        return ExecutePath.Result(success=True, message="Path concluído")

    # ---------- feedback / chegada ----------
    def _on_feedback_tick(self):
        if not (self._active and self._gh and self._points):
            return

        # mesmo em pausa geramos feedback (o arbiter está segurando o movimento)
        try:
            tr = self.tf.lookup_transform(self.global_frame, self.base_link_frame, rclpy.time.Time())
        except Exception:
            return

        x = float(tr.transform.translation.x)
        y = float(tr.transform.translation.y)

        # suavizar avanço do índice para feedback
        idx_now = nearest_index(self._points, (x, y))
        if not self._paused and idx_now > self._last_idx:
            step = min(idx_now - self._last_idx, self.lookahead_idx_cap)
            self._last_idx += step

        rem = remaining_distance(self._points, self._last_idx, (x, y))

        fb = ExecutePath.Feedback()
        fb.current_index = int(self._last_idx)
        fb.total_points = int(self._total_points)
        fb.remaining_distance = float(rem)
        try:
            self._gh.publish_feedback(fb)
        except Exception:
            pass

        # chegada (critério simples no último ponto)
        if dist_xy((x, y), self._points[-1]) <= self.reach_tol:
            self.get_logger().info("[orchestrator] chegada detectada.")
            # peça para o arbiter parar limpo
            self._send_ctrl("CANCEL")
            self._active = False


# ---------------- main ----------------
def main():
    rclpy.init()
    node = ExecutePathOrchestrator()
    exec = MultiThreadedExecutor()
    exec.add_node(node)
    try:
        exec.spin()
    except KeyboardInterrupt:
        pass
    finally:
        exec.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
