#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist

class StmLogger(Node):
    def __init__(self):
        super().__init__('stm_logger')

        # ===== 你 STM 上來的 topic =====
        self.create_subscription(Int32, '/mode_up', self.cb_mode_up, 10)
        self.create_subscription(Int32, '/a_stm_comm', self.cb_a_comm, 10)

        # ===== 你 STM 下去的 topic =====
        self.create_subscription(Int32, '/mode_down', self.cb_mode_down, 10)
        self.create_subscription(Twist, '/cmd_vel', self.cb_cmd_vel, 10)

        self.get_logger().info("STM Logger ready.")

    # ====== callbacks ======
    def cb_mode_up(self, msg: Int32):
        self.get_logger().info(f"[STM -> ROS] /mode_up : {msg.data}")

    def cb_a_comm(self, msg: Int32):
        self.get_logger().info(f"[STM -> ROS] /a_stm_comm : {msg.data}")

    def cb_mode_down(self, msg: Int32):
        self.get_logger().info(f"[ROS -> STM] /mode_down : {msg.data}")
        
    def cb_cmd_vel(self, msg):
        self.get_logger().info(
            f"[ROS->STM] /cmd_vel -> vx={msg.linear.x:.3f}, vy={msg.linear.y:.3f}, wz={msg.angular.z:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = StmLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
