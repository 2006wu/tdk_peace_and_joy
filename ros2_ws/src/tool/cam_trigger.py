#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial, threading

class ModeBridge(Node):
    def __init__(self):
        super().__init__('mode_bridge')
        self.pub_mode = self.create_publisher(Int32, 'mode', 10)

        self.port = '/dev/ttyACM0'
        self.baud = 115200
        self.ser = serial.Serial(self.port, self.baud, timeout=0.1)

        self.buf = bytearray()
        self.thread = threading.Thread(target=self.reader, daemon=True)
        self.thread.start()

    def reader(self):
        while rclpy.ok():
            data = self.ser.read(128)
            if data:
                self.buf.extend(data)
                while b'\n' in self.buf:
                    line, _, rest = self.buf.partition(b'\n')
                    self.buf = bytearray(rest)
                    self.handle_line(line.decode(errors='ignore').strip())

    def handle_line(self, s):
        parts = s.split()
        if len(parts) == 2 and parts[0].upper() == "MODE":
            try:
                value = int(parts[1])
                msg = Int32()
                msg.data = value
                self.pub_mode.publish(msg)
                self.get_logger().info(f'Published mode={value}')
            except ValueError:
                self.get_logger().warn(f'Invalid MODE line: {s}')

def main():
    rclpy.init()
    node = ModeBridge()
    try:
        rclpy.spin(node)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
