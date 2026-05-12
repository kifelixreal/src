#!/usr/bin/env python3
"""
RobotHealthMonitor

Monitora a saúde de todos os subsistemas do robô e publica um JSON
no tópico /robot_health a 1Hz.

Fontes de dados:
  - ZED:          /zed/zed_node/status/health  (zed_msgs/HealthStatusStamped)
                  /zed/zed_node/status/heartbeat (zed_msgs/Heartbeat)
  - LiDAR:        /scan (sensor_msgs/LaserScan)
  - EKF:          /diagnostics (diagnostic_msgs/DiagnosticArray)
  - Nav2:         /bond (bond/msg/Status) — por componente
                  /diagnostics — lifecycle manager (fallback)
  - Drive:        /ui/drive_status (std_msgs/String JSON)
  - Localização:  /amcl_pose (geometry_msgs/PoseWithCovarianceStamped)

Payload publicado em /robot_health:
{
  "t": float,
  "overall_ok": bool,
  "zed": { "ok": bool, "heartbeat_ok": bool, "low_image_quality": bool,
           "low_lighting": bool, "low_depth_reliability": bool,
           "low_motion_sensors_reliability": bool },
  "lidar": { "ok": bool, "hz": float|null },
  "ekf":   { "ok": bool, "hz": float|null },
  "nav2": {
    "ok": bool,
    "components": {
      "controller_server": bool,
      "planner_server":    bool,
      "bt_navigator":      bool,
      "map_server":        bool,
      "amcl":              bool,
      "velocity_smoother": bool,
      "behavior_server":   bool,
      "smoother_server":   bool,
      "waypoint_follower": bool
    }
  },
  "drive": { "ok": bool, "axis0": {...}, "axis1": {...} },
  "localization": {
    "ok": bool,
    "status": "INITIALIZING" | "LOCALIZED" | "LOST",
    "var_x": float|null, "var_y": float|null, "var_yaw": float|null
  }
}
"""

import time
import json
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
)

from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from bond.msg import Status as BondStatus

from zed_msgs.msg import HealthStatusStamped, Heartbeat


def to_int(v, default=0):
    try:
        return int(v)
    except Exception:
        return default


def _median(values):
    s = sorted(values)
    n = len(s)
    mid = n // 2
    return s[mid] if n % 2 else (s[mid - 1] + s[mid]) / 2.0


# Componentes Nav2 monitorados via bond.
# "critical" = sua falha derruba nav2_ok e overall_ok
# os não-críticos são reportados mas não derrubam o overall
NAV2_COMPONENTS = {
    "controller_server": True,
    "planner_server":    True,
    "bt_navigator":      True,
    "map_server":        True,
    "amcl":              True,
    "velocity_smoother": True,
    "behavior_server":   False,
    "smoother_server":   False,
    "waypoint_follower": False,
}

# Tolerância extra sobre o heartbeat_timeout do bond (segundos).
# O bond publica a 10Hz; adicionamos margem para não termos falsos alarmes.
BOND_TTL_MARGIN = 1.5


class RobotHealthMonitor(Node):

    def __init__(self):
        super().__init__("robot_health_monitor")

        # ── parâmetros ────────────────────────────────────────────────
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("pub_topic",  "/robot_health")

        self.declare_parameter("zed_ttl_sec",   3.0)
        self.declare_parameter("lidar_ttl_sec", 2.0)
        self.declare_parameter("ekf_ttl_sec",   3.0)
        self.declare_parameter("drive_ttl_sec", 3.0)
        self.declare_parameter("amcl_ttl_sec",  3.0)

        # Thresholds de covariância — mesmos do localization_manager
        self.declare_parameter("loc_max_var_x",   0.4)
        self.declare_parameter("loc_max_var_y",   0.4)
        self.declare_parameter("loc_max_var_yaw", 0.15)
        self.declare_parameter("loc_window_s",    10.0)
        self.declare_parameter("loc_min_samples", 5)

        self.scan_topic      = self.get_parameter("scan_topic").value
        self.pub_topic       = self.get_parameter("pub_topic").value
        self.zed_ttl         = self.get_parameter("zed_ttl_sec").value
        self.lidar_ttl       = self.get_parameter("lidar_ttl_sec").value
        self.ekf_ttl         = self.get_parameter("ekf_ttl_sec").value
        self.drive_ttl       = self.get_parameter("drive_ttl_sec").value
        self.amcl_ttl        = self.get_parameter("amcl_ttl_sec").value
        self.loc_max_var_x   = self.get_parameter("loc_max_var_x").value
        self.loc_max_var_y   = self.get_parameter("loc_max_var_y").value
        self.loc_max_var_yaw = self.get_parameter("loc_max_var_yaw").value
        self.loc_window_s    = self.get_parameter("loc_window_s").value
        self.loc_min_samples = self.get_parameter("loc_min_samples").value

        # ── publisher ─────────────────────────────────────────────────
        self.pub = self.create_publisher(String, self.pub_topic, 10)

        # ── estado interno ────────────────────────────────────────────

        self.zed = {
            "last_health_t":                  None,
            "last_heartbeat_t":               None,
            "low_image_quality":              False,
            "low_lighting":                   False,
            "low_depth_reliability":          False,
            "low_motion_sensors_reliability": False,
        }

        self.lidar = {
            "last_scan_t": None,
            "hz":          None,
            "_prev_t":     None,
            "_ema_hz":     None,
        }

        self.ekf = {
            "last_ok_t": None,
            "hz":        None,
        }

        self.drive = {
            "last_msg_t": None,
            "connected":  False,
            "axis0":      {},
            "axis1":      {},
        }

        # Nav2 via bond — para cada componente guardamos:
        #   last_t:   timestamp do último heartbeat recebido
        #   timeout:  heartbeat_timeout reportado pelo próprio bond
        self.nav2_bonds: dict = {
            name: {"last_t": None, "timeout": 4.0}
            for name in NAV2_COMPONENTS
        }

        # Localização
        self.amcl_hist: deque = deque()
        self.amcl_last_t: float = None

        # ── subscriptions ─────────────────────────────────────────────

        self.create_subscription(
            HealthStatusStamped,
            "/zed/zed_node/status/health",
            self._on_zed_health,
            10,
        )

        self.create_subscription(
            Heartbeat,
            "/zed/zed_node/status/heartbeat",
            self._on_zed_heartbeat,
            10,
        )

        self.create_subscription(
            LaserScan,
            self.scan_topic,
            self._on_scan,
            10,
        )

        self.create_subscription(
            DiagnosticArray,
            "/diagnostics",
            self._on_diag,
            10,
        )

        self.create_subscription(
            String,
            "/ui/drive_status",
            self._on_drive_status,
            10,
        )

        # Bond — monitoramento granular do Nav2
        self.create_subscription(
            BondStatus,
            "/bond",
            self._on_bond,
            10,
        )

        # AMCL pose — QoS TRANSIENT_LOCAL igual ao localization_manager
        qos_amcl = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._on_amcl_pose,
            qos_amcl,
        )

        # ── timer ─────────────────────────────────────────────────────
        self.create_timer(1.0, self._publish_health)

        self.get_logger().info(
            f"[robot_health_monitor] up | pub={self.pub_topic}"
        )

    # ─────────────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────────────

    def _on_zed_health(self, msg: HealthStatusStamped):
        now = time.time()
        self.zed["last_health_t"]                    = now
        self.zed["low_image_quality"]                = bool(msg.low_image_quality)
        self.zed["low_lighting"]                     = bool(msg.low_lighting)
        self.zed["low_depth_reliability"]            = bool(msg.low_depth_reliability)
        self.zed["low_motion_sensors_reliability"]   = bool(msg.low_motion_sensors_reliability)

    def _on_zed_heartbeat(self, msg: Heartbeat):
        self.zed["last_heartbeat_t"] = time.time()

    def _on_scan(self, msg: LaserScan):
        now = time.time()
        self.lidar["last_scan_t"] = now
        prev = self.lidar["_prev_t"]
        if prev is not None:
            dt = now - prev
            if dt > 1e-3:
                hz_inst = 1.0 / dt
                ema     = self.lidar["_ema_hz"]
                self.lidar["_ema_hz"] = (
                    hz_inst if ema is None
                    else 0.2 * hz_inst + 0.8 * ema
                )
                self.lidar["hz"] = round(self.lidar["_ema_hz"], 2)
        self.lidar["_prev_t"] = now

    def _on_drive_status(self, msg: String):
        now = time.time()
        try:
            data = json.loads(msg.data)
            self.drive["last_msg_t"] = now
            self.drive["connected"]  = bool(data.get("connected", False))
            self.drive["axis0"]      = data.get("axis0", {})
            self.drive["axis1"]      = data.get("axis1", {})
        except Exception as e:
            self.get_logger().warn(f"[drive_status] parse error: {e}")

    def _on_diag(self, msg: DiagnosticArray):
        """
        Mantém apenas o monitoramento do EKF via /diagnostics.
        Nav2 agora é monitorado via /bond.
        """
        now = time.time()
        for st in msg.status:
            name    = (st.name    or "").lower()
            message = (st.message or "").lower()
            ok      = (to_int(st.level, 0) == 0)

            if "ekf_odom" in name:
                if "functioning properly" in message and ok:
                    self.ekf["last_ok_t"] = now
                for kv in st.values:
                    if (kv.key or "").lower() == "actual frequency (hz)":
                        try:
                            self.ekf["hz"] = round(float(kv.value), 2)
                        except Exception:
                            pass

    def _on_bond(self, msg: BondStatus):
        """
        Atualiza o timestamp do último heartbeat de cada componente Nav2.
        O bond publica a ~10Hz por componente com active=True enquanto vivo.
        Quando um nó morre, para de publicar — detectamos pelo TTL.
        """
        component = msg.id  # ex: "controller_server", "planner_server", ...

        if component not in self.nav2_bonds:
            return  # componente não monitorado, ignora

        if not msg.active:
            # Bond explicitamente marcado como inativo — falha imediata
            self.nav2_bonds[component]["last_t"] = None
            return

        now = time.time()
        self.nav2_bonds[component]["last_t"]  = now
        # Atualiza o timeout com o valor reportado pelo próprio bond
        self.nav2_bonds[component]["timeout"] = float(msg.heartbeat_timeout)

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped):
        """
        Janela deslizante de covariâncias AMCL.
        Índices na matriz 6x6 flat: [0]=var_x, [7]=var_y, [35]=var_yaw
        """
        now = time.time()
        self.amcl_last_t = now

        cov = msg.pose.covariance
        self.amcl_hist.append((now, float(cov[0]), float(cov[7]), float(cov[35])))

        while self.amcl_hist and (now - self.amcl_hist[0][0]) > self.loc_window_s:
            self.amcl_hist.popleft()

    # ─────────────────────────────────────────────────────────────────
    # Lógica de localização
    # ─────────────────────────────────────────────────────────────────

    def _localization_status(self):
        now = time.time()

        if self.amcl_last_t is None or (now - self.amcl_last_t) > self.amcl_ttl:
            return False, "INITIALIZING", None, None, None

        if len(self.amcl_hist) < self.loc_min_samples:
            return False, "INITIALIZING", None, None, None

        mx   = round(_median([e[1] for e in self.amcl_hist]), 4)
        my   = round(_median([e[2] for e in self.amcl_hist]), 4)
        myaw = round(_median([e[3] for e in self.amcl_hist]), 4)

        localized = (
            mx   <= self.loc_max_var_x   and
            my   <= self.loc_max_var_y   and
            myaw <= self.loc_max_var_yaw
        )

        return localized, ("LOCALIZED" if localized else "LOST"), mx, my, myaw

    # ─────────────────────────────────────────────────────────────────
    # Lógica Nav2 via bond
    # ─────────────────────────────────────────────────────────────────

    def _nav2_status(self):
        """
        Retorna (nav2_ok, components_dict).

        Para cada componente:
          ok = recebeu heartbeat dentro de (heartbeat_timeout + BOND_TTL_MARGIN)

        nav2_ok = todos os componentes CRÍTICOS estão ok.
        """
        now = time.time()
        components = {}

        for name, critical in NAV2_COMPONENTS.items():
            bond = self.nav2_bonds[name]
            last_t  = bond["last_t"]
            timeout = bond["timeout"] + BOND_TTL_MARGIN

            ok = last_t is not None and (now - last_t) <= timeout
            components[name] = ok

        # nav2_ok = todos os críticos ok
        nav2_ok = all(
            components[name]
            for name, critical in NAV2_COMPONENTS.items()
            if critical
        )

        return nav2_ok, components

    # ─────────────────────────────────────────────────────────────────
    # Publicação
    # ─────────────────────────────────────────────────────────────────

    def _publish_health(self):
        now = time.time()

        def ttl_ok(last_t, ttl):
            return last_t is not None and (now - last_t) <= ttl

        # ZED
        zed_health_ok    = ttl_ok(self.zed["last_health_t"],    self.zed_ttl)
        zed_heartbeat_ok = ttl_ok(self.zed["last_heartbeat_t"], self.zed_ttl)
        zed_flags_ok     = (
            not self.zed["low_image_quality"] and
            not self.zed["low_motion_sensors_reliability"]
        )
        zed_ok = zed_health_ok and zed_heartbeat_ok and zed_flags_ok

        # LiDAR
        lidar_ok = ttl_ok(self.lidar["last_scan_t"], self.lidar_ttl)

        # EKF
        ekf_ok = ttl_ok(self.ekf["last_ok_t"], self.ekf_ttl)

        # Nav2
        nav2_ok, nav2_components = self._nav2_status()

        # Drive
        drive_msg_ok = ttl_ok(self.drive["last_msg_t"], self.drive_ttl)
        a0 = self.drive["axis0"]
        a1 = self.drive["axis1"]
        axis0_ok = a0.get("connected", False) and not a0.get("has_error", False)
        axis1_ok = a1.get("connected", False) and not a1.get("has_error", False)
        drive_ok = drive_msg_ok and axis0_ok and axis1_ok

        # Localização
        loc_ok, loc_status, var_x, var_y, var_yaw = self._localization_status()
        loc_blocks = (loc_status == "LOST")

        # Overall
        overall_ok = (
            zed_ok and lidar_ok and ekf_ok and
            nav2_ok and drive_ok and not loc_blocks
        )

        payload = {
            "t":          now,
            "overall_ok": overall_ok,

            "zed": {
                "ok":                            zed_ok,
                "heartbeat_ok":                  zed_heartbeat_ok,
                "low_image_quality":             self.zed["low_image_quality"],
                "low_lighting":                  self.zed["low_lighting"],
                "low_depth_reliability":         self.zed["low_depth_reliability"],
                "low_motion_sensors_reliability": self.zed["low_motion_sensors_reliability"],
            },

            "lidar": {
                "ok": lidar_ok,
                "hz": self.lidar["hz"],
            },

            "ekf": {
                "ok": ekf_ok,
                "hz": self.ekf["hz"],
            },

            "nav2": {
                "ok":         nav2_ok,
                "components": nav2_components,
            },

            "drive": {
                "ok":    drive_ok,
                "axis0": a0,
                "axis1": a1,
            },

            "localization": {
                "ok":      loc_ok,
                "status":  loc_status,
                "var_x":   var_x,
                "var_y":   var_y,
                "var_yaw": var_yaw,
            },
        }

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = RobotHealthMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()