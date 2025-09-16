#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from functools import partial

class StmStateLogger(Node):
    def __init__(self):
        super().__init__('stm_state_logger')

        # 需要監測的五個 Int32 topic
        self.topics = [
            ('/mode_up',     Int32),
            ('/mode_down',   Int32),
            ('/reset',       Int32),
            ('/grab_cmd',    Int32),
            ('/grab_status', Int32),
        ]

        # 初始值（啟動就先印一次）
        defaults = {
            '/mode_up': 0,
            '/mode_down': 0,
            '/reset': 1,
            '/grab_cmd': 0,
            '/grab_status': 0,
        }
        self.last_values = defaults.copy()
        for name, val in self.last_values.items():
            self.get_logger().info(f"[INIT] {name}: {val}")

        # /cmd_vel 追蹤 vx, vy, wz
        self.last_cmd = (0.0, 0.0, 0.0)  # (vx, vy, wz)
        self.get_logger().info(f"[INIT] /cmd_vel: vx=0.00, vy=0.00, wz=0.00")

        # 訂閱 Int32 類型
        for name, msg_type in self.topics:
            self.create_subscription(
                msg_type,
                name,
                partial(self._on_int32, name=name),
                10
            )

        # 訂閱 /cmd_vel (Twist)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)

    def _on_int32(self, msg: Int32, name: str):
        prev = self.last_values.get(name)
        cur = int(msg.data)
        if cur != prev:
            self.get_logger().info(f"[CHANGED] {name}: {prev} -> {cur}")
            self.last_values[name] = cur
        # 相同不重印

    def _on_cmd_vel(self, msg: Twist):
        vx = float(msg.linear.x)
        vy = float(msg.linear.y)
        wz = float(msg.angular.z)

        pvx, pvy, pwz = self.last_cmd
        # 僅在任一分量改變時印出（直接比較即可；若想避免微小浮點誤差可加容差）
        if (vx, vy, wz) != (pvx, pvy, pwz):
            self.get_logger().info(
                f"[CHANGED] /cmd_vel: "
                f"(vx={pvx:.2f}, vy={pvy:.2f}, wz={pwz:.2f}) -> "
                f"(vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f})"
            )
            self.last_cmd = (vx, vy, wz)

def main(args=None):
    rclpy.init(args=args)
    node = StmStateLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
