# initial_pose_pub.py  (executável no seu pacote)
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import math, time

def q_from_yaw(y):
    import geometry_msgs.msg as gm
    q = gm.Quaternion()
    q.z = math.sin(y/2); q.w = math.cos(y/2); return q

class InitialPosePub(Node):
    def __init__(self):
        super().__init__('initial_pose_pub')
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.timer = self.create_timer(3.0, self.once)  # publica após 3s
        self.done = False
    def once(self):
        if self.done: return
        x = float(self.get_parameter('x').value)
        y = float(self.get_parameter('y').value)
        yaw = float(self.get_parameter('yaw').value)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation = q_from_yaw(yaw)
        # covariâncias iniciais (mais largo = convergência mais rápida/robusta)
        msg.pose.covariance[0] = 0.25   # var x (0.5 m^2)
        msg.pose.covariance[7] = 0.25   # var y
        msg.pose.covariance[35]= (0.35**2)  # var yaw (~20°)
        self.pub.publish(msg)
        self.get_logger().info(f'Initial pose published: ({x:.2f},{y:.2f},{yaw:.2f} rad)')
        self.done = True

def main():
    rclpy.init(); n = InitialPosePub(); rclpy.spin(n)

if __name__ == '__main__': main()
