#!/usr/bin/env python3
import math, time
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformListener


# ====================== utils ======================
def yaw_from_quat_xyzw(x, y, z, w) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

def dist(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])

def path_to_pts(path: Path) -> List[Tuple[float, float]]:
    return [(p.pose.position.x, p.pose.position.y) for p in path.poses]

def nearest_idx(pts: List[Tuple[float, float]], p: Tuple[float, float]) -> int:
    if not pts: return 0
    best_i, best_d = 0, 1e18
    for i, xy in enumerate(pts):
        d = dist(xy, p)
        if d < best_d:
            best_i, best_d = i, d
    return best_i


# ====================== ARBITER LOCAL ======================
class ArbiterLocal(Node):
    """
    Entradas:
      - /path_resampled  (Path)           -> da Action do orquestrador (latched)
      - /vfh_subgoal     (PoseStamped)    -> subgoal do VFH (desvio local)
      - TF map->base_link

    Saídas:
      - /subgoal   (PoseStamped)          -> alvo único para seu controller (compatível)
      - /goal_cmd  (PoseStamped)          -> opcional espelho do comando (debug)

    Regras:
      1) Se há subgoal recente do VFH (sticky), prioriza VFH.
      2) Senão, escolhe alvo do path via lookahead por distância acumulada.
      3) Nunca publica alvo “colado” (min_goal_cmd_dist); se ocorrer, avança índice (anti-thrashing).
      4) Republica periodicamente o alvo (reissue_hz).
    """

    def __init__(self):
        super().__init__('arbiter_local')

        # ------- parâmetros -------
        self.declare_parameter('global_frame', 'map')
        # self.declare_parameter('base_link_frame', 'zed_camera_link')
        self.declare_parameter('base_link_frame', 'base_link')

        self.declare_parameter('path_topic', '/path')
        self.declare_parameter('vfh_subgoal_topic', '/vfh_subgoal')  # <- evite colisão com /subgoal do controller
        self.declare_parameter('controller_cmd_topic', '/goal_cmd')   # <- seu controller já lê /subgoal
        self.declare_parameter('mirror_goal_cmd_topic', '/mirror_goal_cmd') # opcional

        self.declare_parameter('path_ctrl_topic', '/path_ctrl')   # ### NEW
        self.declare_parameter('reach_tol', 0.3)                       # ### NEW
        self.declare_parameter('halt_on_cancel', True)                  # ### NEW

        # lookahead e robustez
        self.declare_parameter('lookahead_dist', 0.80)         # distância acumulada sobre o path
        self.declare_parameter('min_goal_cmd_dist', 0.30)      # não comandar alvo colado ao robô
        self.declare_parameter('advance_on_close', 0.30)       # avanço permanente do hint se ficar perto demais
        self.declare_parameter('reissue_hz', 5.0)              # re-publicar periodicamente

        # VFH sticky
        self.declare_parameter('subgoal_sticky_s', 0.6)        # histerese do desvio do VFH

        # ler parâmetros
        self.global_frame = self.get_parameter('global_frame').value
        self.base_link_frame = self.get_parameter('base_link_frame').value

        self.path_topic = self.get_parameter('path_topic').value
        self.vfh_subgoal_topic = self.get_parameter('vfh_subgoal_topic').value
        self.controller_cmd_topic = self.get_parameter('controller_cmd_topic').value
        self.mirror_goal_cmd_topic = self.get_parameter('mirror_goal_cmd_topic').value

        self.path_ctrl_topic = self.get_parameter('path_ctrl_topic').value
        self.reach_tol = float(self.get_parameter('reach_tol').value)
        self.halt_on_cancel = bool(self.get_parameter('halt_on_cancel').value)

        self.lookahead_dist = float(self.get_parameter('lookahead_dist').value)
        self.min_goal_cmd_dist = float(self.get_parameter('min_goal_cmd_dist').value)
        self.advance_on_close = float(self.get_parameter('advance_on_close').value)
        self.reissue_hz = float(self.get_parameter('reissue_hz').value)
        self.subgoal_sticky_s = float(self.get_parameter('subgoal_sticky_s').value)

        # ------- TF -------
        self.tf = Buffer(cache_time=Duration(seconds=10.0))
        self.tl = TransformListener(self.tf, self)

        # ------- assinaturas -------
        self.create_subscription(Path, self.path_topic, self._path_cb, 10)
        self.create_subscription(PoseStamped, self.vfh_subgoal_topic, self._vfh_cb, 10)
        self.create_subscription(String, self.path_ctrl_topic, self._ctrl_cb, 10)  # ### NEW

        # ------- publicadores -------
        self.pub_cmd = self.create_publisher(PoseStamped, self.controller_cmd_topic, 10)
        self.pub_cmd_mirror = self.create_publisher(PoseStamped, self.mirror_goal_cmd_topic, 10)

        # ------- estado -------
        self.path_pts: List[Tuple[float, float]] = []
        self.k_hint = 0

        self._vfh_xy: Optional[Tuple[float, float]] = None
        self._vfh_until = 0.0

        self.last_cmd_xy: Optional[Tuple[float, float]] = None
        self.last_cmd_time = 0.0

        self._mission_active = False     # ### NEW
        self._paused = False             # ### NEW

        # timer principal: republicação + decisão
        self.timer = self.create_timer(1.0 / max(1e-3, self.reissue_hz), self._tick)

        self.get_logger().info(
            f"[arbiter_local] up | read {self.path_topic}, {self.vfh_subgoal_topic} | write {self.controller_cmd_topic}"
        )

    # --------------- callbacks ---------------
    def _path_cb(self, msg: Path):
        """Hot-swap do path (novo ou atualizado)."""
        new_pts = path_to_pts(msg)
        # obtém pose atual p/ realinhar k_hint
        try:
            tr = self.tf.lookup_transform(self.global_frame, self.base_link_frame, rclpy.time.Time())
            x = float(tr.transform.translation.x)
            y = float(tr.transform.translation.y)
        except Exception:
            x = y = None

        self.path_pts = new_pts
        if self.path_pts:
            self._mission_active = True   # missão em progresso quando há path
            if x is not None:
                # realinha o hint ao ponto mais próximo; evita saltos
                self.k_hint = nearest_idx(self.path_pts, (x, y))
            else:
                self.k_hint = 0
            self.get_logger().info(f"[arbiter] path carregado: {len(self.path_pts)} pts. k_hint={self.k_hint}")
        else:
            # path vazio -> missão inativa (ex.: cancelamento pelo orquestrador)
            self._mission_active = False
            self.k_hint = 0
            self.get_logger().warn("[arbiter] path vazio recebido -> missão inativa")


    def _vfh_cb(self, msg: PoseStamped):
        self._vfh_xy = (float(msg.pose.position.x), float(msg.pose.position.y))
        self._vfh_until = time.time() + self.subgoal_sticky_s
        # não publica aqui; decide no _tick
        self.get_logger().info(f"[arbiter] VFH subgoal rx: {self._vfh_xy}")

    def _ctrl_cb(self, msg: String):
        """CANCEL / PAUSE / RESUME vindo do orquestrador ou supervisor."""
        cmd = (msg.data or "").strip().upper()
        if cmd == "CANCEL":
            self.get_logger().warn("[airbiter] CANCEL recebido.")
            self._mission_active = False
            self._vfh_xy = None
            if self.halt_on_cancel:
                self._publish_halt()  # manda alvo na pose atual pro controller parar
        elif cmd == "PAUSE":
            self.get_logger().info("[arbiter] PAUSE.")
            self._paused = True
        elif cmd == "RESUME":
            self.get_logger().info("[arbiter] RESUME.")
            self._paused = False
        else:
            self.get_logger().warn(f"[arbiter] comando desconhecido em /mission_ctrl: '{cmd}'")

    # --------------- núcleo ---------------
    def _tick(self):
        # pausado ou sem missão: não publica
        if self._paused or not self._mission_active:
            return

        # 1) pose atual
        try:
            tr = self.tf.lookup_transform(self.global_frame, self.base_link_frame, rclpy.time.Time())
        except Exception:
            return

        x = float(tr.transform.translation.x)
        y = float(tr.transform.translation.y)

        # 2) se não há path, nada a fazer
        if not self.path_pts:
            return

        # 3) término: se já perto do último ponto, encerrar missão
        if dist((x, y), self.path_pts[-1]) <= self.reach_tol:
            self.get_logger().info("[arbiter] missão concluída (reach).")
            self._mission_active = False
            self._vfh_xy = None
            self._publish_halt()  # para limpo
            return

        now = time.time()

        # 4) priorizar VFH (se válido e não colado)
        if self._vfh_xy is not None and now <= self._vfh_until:
            goal_xy = self._vfh_xy
            if dist((x, y), goal_xy) >= self.min_goal_cmd_dist:
                self._publish_cmd(goal_xy)
                return
            # subgoal do VFH está colado – descarta para não “chegar parado”
            self._vfh_xy = None

        # 5) seguir path com lookahead por distância acumulada
        idx = nearest_idx(self.path_pts, (x, y))
        k = max(self.k_hint, idx)

        acc = 0.0
        i = k
        while i < len(self.path_pts) - 1 and acc < self.lookahead_dist:
            acc += dist(self.path_pts[i], self.path_pts[i+1])
            i += 1
        k_target = i
        goal_xy = self.path_pts[k_target]

        # anti-colado
        if dist((x, y), goal_xy) < self.min_goal_cmd_dist:
            j = min(len(self.path_pts) - 1, k_target + 1)
            while j < len(self.path_pts) - 1 and dist((x, y), self.path_pts[j]) < self.min_goal_cmd_dist:
                j += 1
            goal_xy = self.path_pts[j]

            if dist((x, y), goal_xy) < self.advance_on_close:
                self.k_hint = min(j + 1, len(self.path_pts) - 1)

        self._publish_cmd(goal_xy)

    def _publish_cmd(self, xy: Tuple[float, float]):
        # evita flood se for exatamente o mesmo ponto em intervalo muito curto
        if self.last_cmd_xy is not None and dist(self.last_cmd_xy, xy) < 1e-3:
            if (time.time() - self.last_cmd_time) < (1.0 / max(1e-3, self.reissue_hz)) * 0.5:
                return

        stamp = self.get_clock().now().to_msg()
        msg = PoseStamped()
        msg.header = Header(frame_id=self.global_frame, stamp=stamp)
        msg.pose.position.x = float(xy[0])
        msg.pose.position.y = float(xy[1])
        msg.pose.orientation.w = 1.0

        self.pub_cmd.publish(msg)
        if self._vfh_xy is None:
            self.pub_cmd_mirror.publish(msg)

        self.last_cmd_xy = xy
        self.last_cmd_time = time.time()

    def _publish_halt(self):
        """Publica um alvo na pose atual para o controller “parar limpo” sem mudar o controller."""
        try:
            tr = self.tf.lookup_transform(self.global_frame, self.base_link_frame, rclpy.time.Time())
            x = float(tr.transform.translation.x)
            y = float(tr.transform.translation.y)
        except Exception:
            return

        stamp = self.get_clock().now().to_msg()
        msg = PoseStamped()
        msg.header = Header(frame_id=self.global_frame, stamp=stamp)
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.w = 1.0
        self.pub_cmd.publish(msg)
        self.pub_cmd_mirror.publish(msg)

# ====================== main ======================
def main():
    rclpy.init()
    node = ArbiterLocal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
