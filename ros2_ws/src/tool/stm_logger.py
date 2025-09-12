#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from functools import partial

class StmStateLogger(Node):
    def __init__(self):
        super().__init__('stm_state_logger')

        # 需要監測的五個 topic（皆為 std_msgs/Int32）
        self.topics = [
            ('/mode_up',     Int32),
            ('/mode_down',   Int32),
            ('/reset',       Int32),
            ('/grab_cmd',    Int32),
            ('/grab_status', Int32),
        ]

        # 紀錄前一次的數值（None 表示尚未收到）
        self.last_values = {name: None for name, _ in self.topics}

        # 逐一訂閱，將 topic 名字綁到同一個 callback
        for name, msg_type in self.topics:
            self.create_subscription(
                msg_type,
                name,
                partial(self._on_int32, name=name),
                10
            )

        self.get_logger().info("STM State Logger ready. (log-on-change)")

    def _on_int32(self, msg: Int32, name: str):
        prev = self.last_values[name]
        cur = int(msg.data)

        # 第一次或數值有變動才列印
        if prev is None:
            self.get_logger().info(f"[INIT] {name}: {cur}")
            self.last_values[name] = cur
            return

        if cur != prev:
            self.get_logger().info(f"[CHANGED] {name}: {prev} -> {cur}")
            self.last_values[name] = cur
        # 若相同就不列印

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
