import sys
import serial
from PyQt6.QtWidgets import QApplication, QMainWindow
from robotui import Ui_MainWindow

# --- Cấu hình cổng serial ---
SERIAL_PORT = 'COM5'  # ⚠️ Thay COM4 bằng cổng thực tế của bạn
BAUD_RATE = 115200

class RobotArmController(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # Khởi tạo góc ban đầu cho 6 khớp (servo 1–6)
        self.servo_angles = [90] * 6

        # Kết nối nút tăng/giảm cho từng khớp
        self.ui.inc1.clicked.connect(lambda: self.move_servo(0, 5))   # Joint 1 → Servo 1
        self.ui.des1.clicked.connect(lambda: self.move_servo(0, -5))
        self.ui.inc2.clicked.connect(lambda: self.move_servo(1, 5))   # Joint 2 → Servo 2
        self.ui.des2.clicked.connect(lambda: self.move_servo(1, -5))
        self.ui.inc3.clicked.connect(lambda: self.move_servo(2, 5))   # Joint 3 → Servo 3
        self.ui.des3.clicked.connect(lambda: self.move_servo(2, -5))
        self.ui.inc4.clicked.connect(lambda: self.move_servo(3, 5))   # Joint 4 → Servo 4
        self.ui.des4.clicked.connect(lambda: self.move_servo(3, -5))
        self.ui.inc5.clicked.connect(lambda: self.move_servo(4, 5))   # Joint 5 → Servo 5
        self.ui.des5.clicked.connect(lambda: self.move_servo(4, -5))
        self.ui.inc6.clicked.connect(lambda: self.move_servo(5, 5))   # Joint 6 → Servo 6
        self.ui.des6.clicked.connect(lambda: self.move_servo(5, -5))

        # Kết nối Serial
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            print("✅ Đã kết nối Serial.")
        except Exception as e:
            print(f"❌ Lỗi mở Serial: {e}")
            self.ser = None

    def move_servo(self, index, delta):
        # Giới hạn từ 0 đến 180 độ
        self.servo_angles[index] = max(0, min(180, self.servo_angles[index] + delta))

        # Chuyển đổi sang microseconds
        pulse_width = int(500 + (2000 * self.servo_angles[index] / 180))
        servo_id = index + 1  # Servo ID = Joint index + 1

        command = f"#{servo_id}P{pulse_width}T200\r\n"
        print(f"📤 Gửi lệnh: {command.strip()}")
        if self.ser:
            self.ser.write(command.encode())

        # Cập nhật giao diện hiển thị góc hiện tại
        try:
            if index == 4:
                self.ui.line_j3_val_3.setText(str(self.servo_angles[index]))  # Join 5
            elif index == 5:
                self.ui.line_j4_val_5.setText(str(self.servo_angles[index]))  # Join 6
            else:
                getattr(self.ui, f'line_j{index+1}_val').setText(str(self.servo_angles[index]))
        except AttributeError:
            pass

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = RobotArmController()
    window.show()
    sys.exit(app.exec())