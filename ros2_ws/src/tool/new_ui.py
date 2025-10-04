# 2006wu_08_14 - UI Panel with /reset publisher + Topic Monitor (Continuous Publish)
#!/usr/bin/env python3
import sys, time, threading
from PyQt5 import QtWidgets, QtCore
from importlib import import_module

# ====== UI 參數 ======
CHECKPOINTS = [
    ("Start A", [(11, "A1", "1"), (21, "A2", "q")]),
    ("Reset B", [(12, "B1", "2"), (22, "B2", "w")]),
    ("Reset C", [(13, "C1", "3"), (23, "C2", "e")]),
    ("Reset D", [(14, "D1", "4"), (24, "D2", "r")]),
]

UPDATE_MS = 200   # 監控畫面刷新
RESET_MS  = 100   # reset 發送間隔 (ms)

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
    ("/wheel_odom", "geometry_msgs.msg.Point"),
    ("/lift_cmd", "std_msgs.msg.Int32"),
    ("/lift_status", "std_msgs.msg.Int32"),
    ("/toy_cmd", "std_msgs.msg.Int32"),
    ("/toy_status", "std_msgs.msg.Int32"),
]

# ====== ROS 嘗試載入 ======
ROS_OK = True
try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Int32
except Exception:
    ROS_OK = False


class RosClient(Node if ROS_OK else object):
    def __init__(self):
        if ROS_OK:
            super().__init__("ui_topic_monitor")
            self._lock = threading.Lock()
            self.latest_topics = {}

            for name, type_str in TOPICS_TO_MONITOR:
                module_name, msg_name = type_str.rsplit(".", 1)
                msg_module = import_module(module_name)
                msg_type = getattr(msg_module, msg_name)
                self.create_subscription(
                    msg_type, name,
                    lambda msg, n=name: self._cb_any(n, msg),
                    10,
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
        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(12)

        # ---- 上方 Topic 狀態顯示 ----
        self.monitor_box = QtWidgets.QTextEdit()
        self.monitor_box.setReadOnly(True)
        self.monitor_box.setStyleSheet("""
            QTextEdit {
                font-size: 60px;
                color: #00ff00;
                background-color: #000000;
            }
        """)
        self.monitor_box.setMinimumHeight(200)
        root.addWidget(self.monitor_box, 2)

        # ---- 下方 Reset 按鈕區 ----
        grid = QtWidgets.QGridLayout()
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setSpacing(12)

        self._hotkey_map = {}
        for row, (group_name, buttons) in enumerate(CHECKPOINTS):
            for j, (cid, cname, hk) in enumerate(buttons):
                btn = QtWidgets.QPushButton(cname)
                btn.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
                btn.setMinimumSize(240, 350)
                btn.setStyleSheet("""
                QPushButton {
                    font-size: 60px;
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
                grid.addWidget(btn, row, j)
                self._hotkey_map[hk] = (cid, cname, btn)

        button_panel = QtWidgets.QWidget()
        button_panel.setLayout(grid)
        root.addWidget(button_panel, 1)

        self.resize(1200, 900)

        # ---- ROS 啟動 ----
        self.ros_node = None
        self.spin_thread = None
        self.reset_pub = None
        if ROS_OK:
            try:
                rclpy.init(args=None)
                self.ros_node = RosClient()
                self.spin_thread = RosSpinThread(self.ros_node)
                self.spin_thread.start()
                self.reset_pub = self.ros_node.create_publisher(Int32, "/reset", 10)
                self.mode_down_pub = self.ros_node.create_publisher(Int32, "/mode_down", 10)
                self._mode_up_timer = None   # ✅ 加一個 timer 變數
                self._pending_trigger = None   # ✅ 記錄 A2 或 B2
                self._mode_down_loop_timer = None

            except Exception:
                self.ros_node = None

        # ---- 定時刷新監控 ----
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh_monitor)
        self.timer.start(UPDATE_MS)

        # ---- 定時發送 reset ----
        self.reset_value = 0
        self.reset_timer = QtCore.QTimer(self)
        self.reset_timer.timeout.connect(self._publish_reset_loop)
        self.reset_timer.start(RESET_MS)
        self.toggle_map = {}  # ✅ 加這行

    def refresh_monitor(self):
        if self.ros_node is None:
            self.monitor_box.setText("❌ ROS not available")
            return
        data_all = self.ros_node.get_all_latest()
        lines = []
        # ✅ 偵測 /mode_up
        if "mode_up" in data_all:
            msg, age = data_all["mode_up"]
            if hasattr(msg, "data"):
                # 只有 B2=221 模式才觸發
                if self._pending_trigger is not None:   # ✅ A2 或 B2=221 都能觸發
                    if self._mode_down_loop_timer is None:   # 確保只啟動一次
                        self._mode_down_loop_timer = QtCore.QTimer(self)
                        self._mode_down_loop_timer.timeout.connect(self._send_mode_down)
                        self._mode_down_loop_timer.start(1000)  # ✅ 每 1 秒發一次，可自行改間隔
                        print(f"✅ 收到 /mode_up=1 → 開始持續發 /mode_down (來源 {self._pending_trigger})")


        for tname, (msg, age) in data_all.items():
            if hasattr(msg, "data"):
                value = msg.data
            elif hasattr(msg, "linear") and hasattr(msg, "angular"):
                value = f"vx={msg.linear.x:.2f}, vy={msg.linear.y:.2f}, wz={msg.angular.z:.2f}"
            elif hasattr(msg, "x") and hasattr(msg, "y") and hasattr(msg, "z"):  # Point (/wheel_odom)
                # ✅ 這裡做轉換：x 取負、x/y 轉成 cm
                x_cm = -msg.x * 100.0
                y_cm =  msg.y * 100.0
                value = f"x={x_cm:.1f} cm, y={y_cm:.1f} cm" 
            else:
                value = str(msg)

            color = "#00ff00" if age <= 15 else "#ff0000"
            line = f"<span style='color:{color}'>{tname}: {value}</span>"
            lines.append(line)

        self.monitor_box.setHtml("<br>".join(lines))

    def keyPressEvent(self, e):
        t = e.text()
        if t in self._hotkey_map:
            cid, cname, btn = self._hotkey_map[t]
            self.on_trigger(cid, cname, btn)

    def on_trigger(self, cid, cname, btn):
        orig = btn.styleSheet()
        btn.setStyleSheet(orig + "\nQPushButton { background-color: #2d6cdf; color: white; }")
        QtCore.QTimer.singleShot(150, lambda: btn.setStyleSheet(orig))
        
        
        if cname == "B2":
            if self.toggle_map.get(cname, 0) == 0:
                self.reset_value = 221
                self.toggle_map[cname] = 1
                self._pending_trigger = "B2"   # ✅ 記錄 B2 需要等 mode_up
                print("⏳ B2=221 等待 /mode_up=1 ...")
            else:
                self.reset_value = 222
                self.toggle_map[cname] = 0
                self._pending_trigger = None   # 不做事

        elif cname == "A2":
            self.reset_value = cid
            self._pending_trigger = "A2"       # ✅ 記錄 A2 需要等 mode_up
            print("⏳ A2 等待 /mode_up=1 ...")
        else:
            self.reset_value = cid
            self._pending_trigger = None   # ✅ 清掉等待狀態
            if self._mode_down_loop_timer is not None:
                self._mode_down_loop_timer.stop()
                self._mode_down_loop_timer = None
                print("⏹ 停止持續發送 /mode_down")


    def _publish_reset_loop(self):
        if self.reset_pub is not None:
            msg = Int32()
            msg.data = self.reset_value
            self.reset_pub.publish(msg)

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
    def _send_mode_down(self):
        if self.mode_down_pub is not None:
            msg = Int32()
            msg.data = 6   # ✅ 這裡換成你要的數字
            self.mode_down_pub.publish(msg)


def main():
    app = QtWidgets.QApplication(sys.argv)
    w = Panel()
    w.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
