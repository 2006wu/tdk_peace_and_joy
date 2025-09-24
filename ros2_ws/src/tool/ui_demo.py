# 2006wu_08_14 - UI Panel with /reset publisher + Topic Monitor (Top)
#!/usr/bin/env python3
import sys, time, threading
from PyQt5 import QtWidgets, QtCore
from importlib import import_module

# ====== UI 參數 ======
CHECKPOINTS = [
    (0, "Start A", "1"),
    (1, "Reset B", "2"),
    (2, "Reset C", "3"),
    (3, "Reset D", "4"),
]
UPDATE_MS = 200

# ====== 要監控的 topics ======
TOPICS_TO_MONITOR = [
    ("/start", "std_msgs.msg.Int32"),
    ("/reset", "std_msgs.msg.Int32"),
    ("/mode_up", "std_msgs.msg.Int32"),
    ("/mode_down", "std_msgs.msg.Int32"),
    ("/grab_cmd", "std_msgs.msg.Int32"),
    ("/grab_status", "std_msgs.msg.Int32"),
    ("/bamboo_cmd", "std_msgs.msg.Int32"),
    ("/bamboo_status", "std_msgs.msg.Int32"),
    ("/mission_4_start", "std_msgs.msg.Int32"),
    ("/mission_4_finish", "std_msgs.msg.Int32"),
    ("/cmd_vel", "geometry_msgs.msg.Twist"),
    ("/stair_cmd", "std_msgs.msg.Int32"),
    ("/stair_status", "std_msgs.msg.Int32"),
]

# ====== ROS 嘗試載入 ======
ROS_OK = True
try:
    import rclpy
    from rclpy.node import Node
except Exception:
    ROS_OK = False


class RosClient(Node if ROS_OK else object):
    def __init__(self):
        if ROS_OK:
            super().__init__("ui_topic_monitor")
            self._lock = threading.Lock()
            self.latest_topics = {}   # {topic_name: (msg, timestamp)}

            for name, type_str in TOPICS_TO_MONITOR:
                module_name, msg_name = type_str.rsplit(".", 1)
                msg_module = import_module(module_name)
                msg_type = getattr(msg_module, msg_name)
                self.create_subscription(
                    msg_type, name,
                    lambda msg, n=name: self._cb_any(n, msg),
                    10
                )

    def _cb_any(self, name, msg):
        with self._lock:
            self.latest_topics[name] = (msg, time.time())

    def get_all_latest(self):
        now = time.time()
        with self._lock:
            return {n: (m, now - ts) for n, (m, ts) in self.latest_topics.items()}


class RosSpinThread(QtCore.QThread):
    def __init__(self, node: RosClient):
        super().__init__()
        self.node = node
        self._running = True
    def run(self):
        if not ROS_OK:
            return
        while self._running and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.05)
            time.sleep(0.01)
    def stop(self):
        self._running = False

class Panel(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Mission Panel")
        root = QtWidgets.QVBoxLayout(self)   # 改成上下結構
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(12)

        # ---- 上方 Topic 狀態顯示 ----
        self.monitor_box = QtWidgets.QTextEdit()
        self.monitor_box.setReadOnly(True)
        self.monitor_box.setStyleSheet("""
            QTextEdit {
                font-size: 50px;
                color: #00ff00;
                background-color: #000000;
            }
        """)
        self.monitor_box.setMinimumHeight(100)
        root.addWidget(self.monitor_box, 2)

        # ---- 下方 Reset 按鈕區 ----
        grid = QtWidgets.QGridLayout()
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setSpacing(12)

        self._hotkey_map = {}
        for i, (cid, cname, hk) in enumerate(CHECKPOINTS):
            btn = QtWidgets.QPushButton(cname)
            btn.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
            btn.setMinimumSize(240, 700)
            btn.setStyleSheet("""
            QPushButton {
                font-size: 100px;
                color: white;
                background-color: #333333;
                font-weight: bold;
                border: 2px solid #666666;
                border-radius: 12px;
                padding: 12px;
            }
            QPushButton:pressed { background-color: #444444; }
            """)
            btn.clicked.connect(lambda _, a=cid, b=cname, w=btn: self.on_trigger(a, b, w))
            r, c = divmod(i, 2)
            grid.addWidget(btn, r, c)
            self._hotkey_map[hk] = (cid, cname, btn)

        button_panel = QtWidgets.QWidget()
        button_panel.setLayout(grid)
        root.addWidget(button_panel, 1)

        self.resize(1000, 800)

        # ---- ROS 啟動 ----
        self.ros_node = None
        self.spin_thread = None
        if ROS_OK:
            try:
                rclpy.init(args=None)
                self.ros_node = RosClient()
                self.spin_thread = RosSpinThread(self.ros_node)
                self.spin_thread.start()
            except Exception:
                self.ros_node = None

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh_monitor)
        self.timer.start(UPDATE_MS)

    def refresh_monitor(self):
        if self.ros_node is None:
            self.monitor_box.setText("❌ ROS not available")
            return
        data_all = self.ros_node.get_all_latest()
        lines = []
        for tname, (msg, age) in data_all.items():
            if age > 10:   # 超過 15 秒直接隱藏
                continue

            # 取數值
            if hasattr(msg, "data"):  # std_msgs/Int32
                value = msg.data
            elif hasattr(msg, "linear") and hasattr(msg, "angular"):  # Twist
                value = f"vx={msg.linear.x:.2f}, vy={msg.linear.y:.2f}, wz={msg.angular.z:.2f}"
            else:
                value = str(msg)

            # 判斷顏色
            if age <= 5:
                line = f"<span style='color:#00ff00'>{tname}: {value}</span>"  # 綠色
            else:
                line = f"<span style='color:#888888'>{tname}: {value}</span>"  # 灰色
            lines.append(line)

        self.monitor_box.setHtml("<br>".join(lines))   # 用 HTML 支援顏色

    def keyPressEvent(self, e):
        t = e.text()
        if t in self._hotkey_map:
            cid, cname, btn = self._hotkey_map[t]
            self.on_trigger(cid, cname, btn)

    def on_trigger(self, cid, cname, btn):
        # 點擊高亮
        orig = btn.styleSheet()
        btn.setStyleSheet(orig + "\nQPushButton { background-color: #2d6cdf; color: white; }")
        QtCore.QTimer.singleShot(150, lambda: btn.setStyleSheet(orig))

        if self.ros_node is not None:
            # ✅ 統一呼叫：按下按鈕 (包含 Start) 也會走 reset → 歸零
            self.publish_reset_with_auto_zero(cid + 1)

    def publish_reset_with_auto_zero(self, value: int):
        """發送 /reset = value，然後自動再歸零"""
        from std_msgs.msg import Int32
        pub = self.ros_node.create_publisher(Int32, "/reset", 10)

        # 先發送 value
        msg = Int32()
        msg.data = value
        pub.publish(msg)

        # 0.5 秒後自動歸零
        def reset_back():
            m0 = Int32()
            m0.data = 0
            pub.publish(m0)
        QtCore.QTimer.singleShot(500, reset_back)

    def closeEvent(self, e):
        try:
            if self.spin_thread:
                self.spin_thread.stop()
                self.spin_thread.wait(500)
            if ROS_OK and rclpy.ok():
                if self.ros_node:
                    self.ros_node.destroy_node()
                rclpy.shutdown()
        except Exception:
            pass
        e.accept()

def main():
    app = QtWidgets.QApplication(sys.argv)
    w = Panel()
    w.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
