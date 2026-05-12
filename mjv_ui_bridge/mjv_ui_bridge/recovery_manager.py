#!/usr/bin/env python3
"""
RecoveryManager

Monitora /robot_health e /ui/progress e executa recovery automático
quando detecta falhas críticas persistentes durante uma missão ativa.

Falhas que disparam recovery (stack completo):
  - nav2.ok = false por NAV2_FAIL_THRESHOLD ciclos consecutivos
  - localization.status = LOST por LOC_FAIL_THRESHOLD ciclos consecutivos
  - ekf.ok = false por EKF_FAIL_THRESHOLD ciclos consecutivos

Falhas que só alertam (sem restart):
  - drive.ok = false → cancela missão, para robô, alerta operador
  - zed.ok = false   → alerta operador (por ora)
  - lidar.ok = false → alerta operador (por ora)

Sequência de recovery para falhas críticas:
  1. Publica CANCEL em /mission_ctrl
  2. Publica cmd_vel zero por 1s
  3. Executa: systemctl restart mjv-stack.target
  4. Publica evento em /ui/recovery_event

Proteções:
  - Cooldown de RECOVERY_COOLDOWN_S segundos entre recoveries
  - Só age se há missão ativa (robotState != IDLE/READY)
  - Falha deve ser persistente por N ciclos antes de agir

Tópicos:
  Sub: /robot_health (std_msgs/String JSON)
       /ui/progress  (std_msgs/String JSON)
  Pub: /mission_ctrl     (std_msgs/String)
       /cmd_vel           (geometry_msgs/Twist)
       /ui/recovery_event (std_msgs/String JSON)
"""

import json
import subprocess
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist


# ── Thresholds de falha persistente (ciclos a 1Hz) ───────────────────
NAV2_FAIL_THRESHOLD = 5   # 5s de nav2 down
LOC_FAIL_THRESHOLD  = 8   # 8s de localização LOST
EKF_FAIL_THRESHOLD  = 5   # 5s de EKF down
DRIVE_FAIL_THRESHOLD = 3  # 3s de drive down

# ── Cooldown entre recoveries ────────────────────────────────────────
RECOVERY_COOLDOWN_S = 60.0

# ── Estados de missão que indicam missão ativa ───────────────────────
ACTIVE_MISSION_STATES = {
    "RUNNING", "PAUSED", "RETURNING",
    "AT_STATION", "WAITING",
}

# ── Comando de restart do stack ──────────────────────────────────────
RESTART_CMD = ["systemctl", "restart", "mjv-stack.target"]


class RecoveryManager(Node):

    def __init__(self):
        super().__init__("recovery_manager")

        # ── Parâmetros ────────────────────────────────────────────────
        self.declare_parameter("nav2_fail_threshold",  NAV2_FAIL_THRESHOLD)
        self.declare_parameter("loc_fail_threshold",   LOC_FAIL_THRESHOLD)
        self.declare_parameter("ekf_fail_threshold",   EKF_FAIL_THRESHOLD)
        self.declare_parameter("drive_fail_threshold", DRIVE_FAIL_THRESHOLD)
        self.declare_parameter("recovery_cooldown_s",  RECOVERY_COOLDOWN_S)
        self.declare_parameter("dry_run", False)  # True = não executa restart, só loga

        self.nav2_thresh   = self.get_parameter("nav2_fail_threshold").value
        self.loc_thresh    = self.get_parameter("loc_fail_threshold").value
        self.ekf_thresh    = self.get_parameter("ekf_fail_threshold").value
        self.drive_thresh  = self.get_parameter("drive_fail_threshold").value
        self.cooldown_s    = self.get_parameter("recovery_cooldown_s").value
        self.dry_run       = bool(self.get_parameter("dry_run").value)

        # ── Publishers ────────────────────────────────────────────────
        self.pub_ctrl     = self.create_publisher(String, "/mission_ctrl",      10)
        self.pub_cmdvel   = self.create_publisher(Twist,  "/cmd_vel",           10)
        self.pub_recovery = self.create_publisher(String, "/ui/recovery_event", 10)

        # ── Subscribers ───────────────────────────────────────────────
        self.create_subscription(String, "/robot_health", self._on_health,   10)
        self.create_subscription(String, "/ui/progress",  self._on_progress, 10)

        # ── Estado interno ────────────────────────────────────────────

        # Estado atual da missão
        self._mission_state: str = "IDLE"

        # Contadores de falha consecutiva por subsistema
        self._fail_counts = {
            "nav2":  0,
            "loc":   0,
            "ekf":   0,
            "drive": 0,
        }

        # Timestamp do último recovery executado
        self._last_recovery_t: float = 0.0

        # Flag para não disparar múltiplos recoveries no mesmo evento
        self._recovery_in_progress: bool = False

        if self.dry_run:
            self.get_logger().warn("[recovery_manager] DRY RUN ativo — restart não será executado.")

        self.get_logger().info(
            f"[recovery_manager] up | "
            f"thresholds: nav2={self.nav2_thresh}s loc={self.loc_thresh}s "
            f"ekf={self.ekf_thresh}s drive={self.drive_thresh}s | "
            f"cooldown={self.cooldown_s}s"
        )

    # ─────────────────────────────────────────────────────────────────
    # Callbacks
    # ─────────────────────────────────────────────────────────────────

    def _on_progress(self, msg: String):
        """Acompanha o estado da missão via /ui/progress."""
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        if data.get("event") == "state":
            self._mission_state = data.get("status", "IDLE")

    def _on_health(self, msg: String):
        """
        Recebe /robot_health a 1Hz e avalia falhas persistentes.
        Cada chamada representa um ciclo de avaliação.
        """
        if self._recovery_in_progress:
            return

        try:
            health = json.loads(msg.data)
        except Exception:
            self.get_logger().warn("[recovery_manager] Falha ao parsear /robot_health")
            return

        mission_active = self._mission_state in ACTIVE_MISSION_STATES

        # ── Extrai estados dos subsistemas ────────────────────────────
        nav2_ok  = health.get("nav2",         {}).get("ok",     True)
        ekf_ok   = health.get("ekf",          {}).get("ok",     True)
        drive_ok = health.get("drive",        {}).get("ok",     True)
        loc_status = health.get("localization", {}).get("status", "LOCALIZED")
        loc_lost = (loc_status == "LOST")

        # ZED e LiDAR — só alerta por enquanto
        zed_ok   = health.get("zed",   {}).get("ok", True)
        lidar_ok = health.get("lidar", {}).get("ok", True)

        # ── Atualiza contadores de falha consecutiva ──────────────────
        self._fail_counts["nav2"]  = 0 if nav2_ok  else self._fail_counts["nav2"]  + 1
        self._fail_counts["ekf"]   = 0 if ekf_ok   else self._fail_counts["ekf"]   + 1
        self._fail_counts["drive"] = 0 if drive_ok else self._fail_counts["drive"] + 1
        self._fail_counts["loc"]   = 0 if not loc_lost else self._fail_counts["loc"] + 1

        # ── Log de diagnóstico quando há falha acumulando ─────────────
        for name, count in self._fail_counts.items():
            if count > 0:
                thresh = {
                    "nav2": self.nav2_thresh, "loc": self.loc_thresh,
                    "ekf": self.ekf_thresh,   "drive": self.drive_thresh,
                }[name]
                self.get_logger().warn(
                    f"[recovery_manager] {name} falha há {count}/{thresh} ciclos"
                )

        # ── Alertas para ZED e LiDAR (sem recovery por ora) ──────────
        if not zed_ok:
            self.get_logger().warn("[recovery_manager] ZED degradada — monitorando")
            self._publish_alert("zed_degraded", "ZED com falha — monitorando.")

        if not lidar_ok:
            self.get_logger().warn("[recovery_manager] LiDAR offline — monitorando")
            self._publish_alert("lidar_offline", "LiDAR offline — monitorando.")

        # ── Avalia se deve disparar recovery ─────────────────────────

        # Drive — cancela missão mas não reinicia stack
        if self._fail_counts["drive"] >= self.drive_thresh:
            self.get_logger().error("[recovery_manager] Drive com falha persistente.")
            self._fail_counts["drive"] = 0
            if mission_active:
                self._cancel_mission()
            self._stop_robot()
            self._publish_alert(
                "drive_failure",
                "Falha persistente no drive. Missão cancelada. Verifique o ODrive.",
            )
            return

        # Falhas críticas de navegação — reinicia stack
        nav2_critical  = self._fail_counts["nav2"] >= self.nav2_thresh
        ekf_critical   = self._fail_counts["ekf"]  >= self.ekf_thresh
        loc_critical   = self._fail_counts["loc"]  >= self.loc_thresh

        if nav2_critical or ekf_critical or loc_critical:
            reason = (
                "nav2 offline"       if nav2_critical else
                "EKF offline"        if ekf_critical  else
                "localização perdida"
            )
            subsystem = (
                "nav2" if nav2_critical else
                "ekf"  if ekf_critical  else
                "loc"
            )

            self.get_logger().error(
                f"[recovery_manager] Falha crítica persistente: {reason}. "
                f"Iniciando recovery."
            )

            # Zera contadores para não redisparar imediatamente
            self._fail_counts[subsystem] = 0

            self._execute_recovery(reason=reason, mission_active=mission_active)

    # ─────────────────────────────────────────────────────────────────
    # Ações de recovery
    # ─────────────────────────────────────────────────────────────────

    def _execute_recovery(self, reason: str, mission_active: bool):
        """
        Sequência completa de recovery para falhas críticas de navegação.
        """
        now = time.time()

        # Verifica cooldown
        if (now - self._last_recovery_t) < self.cooldown_s:
            remaining = self.cooldown_s - (now - self._last_recovery_t)
            self.get_logger().warn(
                f"[recovery_manager] Recovery solicitado mas em cooldown "
                f"({remaining:.0f}s restantes). Ignorando."
            )
            return

        self._recovery_in_progress = True
        self._last_recovery_t = now

        self.get_logger().warn(
            f"[recovery_manager] Iniciando recovery — motivo: {reason}"
        )

        # Passo 1: cancela missão se ativa
        if mission_active:
            self.get_logger().info("[recovery_manager] Cancelando missão...")
            self._cancel_mission()
            time.sleep(0.5)

        # Passo 2: para o robô por 1s
        self.get_logger().info("[recovery_manager] Parando robô...")
        self._stop_robot(duration_s=1.0)

        # Passo 3: reinicia o stack
        self.get_logger().info(
            f"[recovery_manager] Reiniciando mjv-stack.target "
            f"({'DRY RUN' if self.dry_run else 'REAL'})..."
        )

        if not self.dry_run:
            try:
                result = subprocess.run(
                    RESTART_CMD,
                    timeout=10.0,
                    capture_output=True,
                    text=True,
                )
                if result.returncode == 0:
                    self.get_logger().info("[recovery_manager] Stack reiniciado com sucesso.")
                else:
                    self.get_logger().error(
                        f"[recovery_manager] Falha no restart: {result.stderr.strip()}"
                    )
            except subprocess.TimeoutExpired:
                self.get_logger().error("[recovery_manager] Timeout no restart do stack.")
            except Exception as e:
                self.get_logger().error(f"[recovery_manager] Erro ao reiniciar stack: {e}")
        else:
            self.get_logger().warn(
                f"[recovery_manager] DRY RUN — comando seria: {' '.join(RESTART_CMD)}"
            )

        # Passo 4: publica alerta para o operador
        self._publish_alert(
            "stack_restarted",
            f"Recovery executado ({reason}). Stack reiniciado. Aguarde o robô inicializar.",
        )

        self._recovery_in_progress = False
        self.get_logger().info("[recovery_manager] Recovery concluído.")

    def _cancel_mission(self):
        """Publica CANCEL em /mission_ctrl."""
        msg = String()
        msg.data = "CANCEL"
        self.pub_ctrl.publish(msg)
        self.get_logger().info("[recovery_manager] CANCEL publicado em /mission_ctrl.")

    def _stop_robot(self, duration_s: float = 1.0):
        """Publica cmd_vel zero por duration_s segundos."""
        twist = Twist()  # todos os campos já são zero por padrão
        end = time.time() + duration_s
        while time.time() < end:
            self.pub_cmdvel.publish(twist)
            time.sleep(0.05)

    def _publish_alert(self, event: str, message: str):
        """Publica evento de recovery em /ui/recovery_event."""
        payload = {
            "event":   event,
            "message": message,
            "t":       time.time(),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub_recovery.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RecoveryManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()