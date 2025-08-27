#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Int32MultiArray
import serial, re, time

class STM32Bridge(Node):
    def __init__(self):
        super().__init__('stm32_bridge_v2')
        self.port = self.declare_parameter('port', '/dev/ttyACM0').get_parameter_value().string_value
        self.baud = self.declare_parameter('baud', 115200).get_parameter_value().integer_value

        self.pub_raw  = self.create_publisher(String, 'stm32_raw', 10)
        self.pub_int  = self.create_publisher(Int32,   'stm32_int', 10)
        self.pub_ints = self.create_publisher(Int32MultiArray, 'stm32_ints', 10)

        self.ser = self._open_serial()

        # 20 Hz poll
        self.timer = self.create_timer(0.05, self.poll)

        # 預留正規表示式（你原來的）
        self.re_pos   = re.compile(r'^POS:(-?\d+(\.\d+)?),(-?\d+(\.\d+)?),(-?\d+(\.\d+)?)$')
        self.re_twist = re.compile(r'^TWIST:(-?\d+(\.\d+)?),(-?\d+(\.\d+)?),(-?\d+(\.\d+)?)$')
        # 整數偵測
        self.re_int_line   = re.compile(r'^-?\d+$')
        self.re_int_tokens = re.compile(r'-?\d+')

    def _open_serial(self):
        while True:
            try:
                ser = serial.Serial(self.port, self.baud, timeout=0.2)
                self.get_logger().info(f'Opened {self.port} @ {self.baud}')
                return ser
            except Exception as e:
                self.get_logger().error(f'Open serial failed: {e}; retry in 1s')
                time.sleep(1.0)

    def poll(self):
        try:
            line = self.ser.readline().decode(errors='ignore').strip()
        except Exception as e:
            self.get_logger().warn(f'serial read error: {e}; reopen...')
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = self._open_serial()
            return

        if not line:
            return

        # 總是發 raw
        raw = String(); raw.data = line
        self.pub_raw.publish(raw)

        # 1) 單一整數（例如： "123" 或 "-45"）
        if self.re_int_line.match(line):
            try:
                val = int(line)
                self.pub_int.publish(Int32(data=val))
            except ValueError:
                pass
            return

        # 2) 多整數以逗號/空白分隔（例如："1,2,-3" 或 "10 -5 8"）
        tokens = self.re_int_tokens.findall(line)
        if tokens and len(tokens) > 1:
            try:
                arr = [int(t) for t in tokens]
                msg = Int32MultiArray()
                msg.data = arr
                self.pub_ints.publish(msg)
                return
            except ValueError:
                pass

        # 3) 其餘格式，之後如果要解析 POS/TWIST 再開下面
        # m = self.re_pos.match(line)
        # if m:
        #     x, y, th = float(m.group(1)), float(m.group(3)), float(m.group(5))
        #     return
        # m = self.re_twist.match(line)
        # if m:
        #     vx, vy, w = float(m.group(1)), float(m.group(3)), float(m.group(5))
        #     return

def main():
    rclpy.init()
    node = STM32Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
