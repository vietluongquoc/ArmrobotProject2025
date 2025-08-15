import sys
import serial
from PyQt5 import QtWidgets
from robot_control import Ui_MainWindow  # file giao diện đã convert từ .ui sang .py

# Cấu hình cổng Serial
SERIAL_PORT = 'COM3'   # Đổi lại theo cổng thực tế của bạn
BAUD_RATE = 115200

class RobotController(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # Kết nối Serial
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            print("✅ Đã kết nối Serial.")
        except serial.SerialException:
            self.ser = None
            print("⚠️ Không thể kết nối Serial.")

        # Danh sách spinbox điều khiển góc
        self.joint_spinboxes = [
            self.ui.spin_J1,
            self.ui.spin_J2,
            self.ui.spin_J3,
            self.ui.spin_J4
        ]

        # Nút tăng góc
        self.buttons_inc = [
            self.ui.btn_inc_J1,
            self.ui.btn_inc_J2,
            self.ui.btn_inc_J3,
            self.ui.btn_inc_J4
        ]

        # Nút giảm góc
        self.buttons_dec = [
            self.ui.btn_dec_J1,
            self.ui.btn_dec_J2,
            self.ui.btn_dec_J3,
            self.ui.btn_dec_J4
        ]

        # Kết nối sự kiện cho nút tăng/giảm
        for i in range(4):
            self.buttons_inc[i].clicked.connect(lambda checked, j=i: self.adjust_joint(j, +5))
            self.buttons_dec[i].clicked.connect(lambda checked, j=i: self.adjust_joint(j, -5))

        # Nút về Home
        self.ui.btn_home.clicked.connect(self.move_home)

        # Nút gửi tất cả giá trị
        self.ui.btn_setting.clicked.connect(self.send_all_joints)

        # Nút On (bật/tắt)
        self.ui.btn_on.clicked.connect(self.toggle_on)

        self.robot_on = False  # Trạng thái bật/tắt

    def send_servo(self, servo_id, angle, speed=500):
        """Gửi lệnh điều khiển servo qua Serial."""
        if self.ser and self.ser.is_open:
            angle = max(0, min(180, angle))
            pulse_width = int(500 + (angle / 180) * 2000)
            cmd = f"#{servo_id+1}P{pulse_width}T{speed}\r\n"
            print(f"Gửi: {cmd.strip()}")
            self.ser.write(cmd.encode('ascii'))
        else:
            print("❌ Serial chưa kết nối.")

    def adjust_joint(self, joint_index, delta):
        """Điều chỉnh góc joint."""
        spin = self.joint_spinboxes[joint_index]
        new_val = spin.value() + delta
        new_val = max(0, min(180, new_val))
        spin.setValue(new_val)
        self.send_servo(joint_index, new_val)

    def send_all_joints(self):
        """Gửi toàn bộ góc hiện tại của các khớp."""
        for i, spin in enumerate(self.joint_spinboxes):
            self.send_servo(i, spin.value())

    def move_home(self):
        """Đưa robot về vị trí Home (90 độ mỗi khớp)."""
        for i, spin in enumerate(self.joint_spinboxes):
            spin.setValue(90)
            self.send_servo(i, 90)

    def toggle_on(self):
        """Bật hoặc tắt robot."""
        self.robot_on = not self.robot_on
        self.ui.btn_on.setText("OFF" if self.robot_on else "ON")
        print("Robot đang:", "BẬT" if self.robot_on else "TẮT")

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = RobotController()
    window.show()
    sys.exit(app.exec_())
