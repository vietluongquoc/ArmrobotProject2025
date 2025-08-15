# codedieukhien.py
import sys
import serial
import serial.tools.list_ports
import numpy as np
from PyQt5 import QtWidgets, QtCore
from robot_control import Ui_MainWindow

# ==== Cấu hình mặc định (sửa nếu cần) ====
DEFAULT_SERIAL_PORT = 'COM3'
BAUD_RATE = 115200

# Bảng bước xung bạn cung cấp (bước cho 10°) -> chia cho 10 => bước / 1°
PULSE_STEP_PER_10DEG = [72.222, 50.0, 116.7, 105.55, 111.11, 105.55]
PULSE_STEP_PER_DEG = [v / 10.0 for v in PULSE_STEP_PER_10DEG]

# Mặc định: xung "zero" (ở đây chọn 1500us làm trung tâm). Nếu servo thực tế khác, chỉnh list sau.
PULSE_ZERO_OFFSET = [1500, 1500, 1500, 1500, 1500, 1500]

# Kích thước khung (theo yêu cầu): L1 = 17, L2 = 13 (đã chỉnh), L3 = 3 (đơn vị cm)
# Khi tính HTM mình sẽ dùng mét (m)
L1_cm = 17.0
L2_cm = 13.0  # <-- đã chỉnh theo yêu cầu
L3_cm = 0

# Chuyển sang mét
L1 = L1_cm * 10
L2 = L2_cm * 10
L3 = L3_cm * 10

# Hàm chuyển góc -> xung cho từng servo
def angle_to_pulse(servo_index: int, angle_deg: float) -> int:
    """Map góc (0..180) sang pulse theo hệ số riêng của từng servo.
       pulse = zero_offset + (angle - 90) * step_per_deg
       (trong đó mình dựa vào trung tâm 90° là offset 1500)"""
    angle = max(0.0, min(180.0, float(angle_deg)))
    step = PULSE_STEP_PER_DEG[servo_index]
    zero = PULSE_ZERO_OFFSET[servo_index]
    # Tính tương đối so với 90°
    pulse = zero + (angle - 90.0) * step
    return int(round(pulse))


class RobotController(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # Serial
        self.ser = None
        self.current_port = None
        self.baud = BAUD_RATE

        # SpinBox góc (tên chính xác từ robot_control.py)
        self.joint_spinboxes = [
            self.ui.spin_J1,
            self.ui.spin_J2,
            self.ui.spin_J3,
            self.ui.spin_J4,
            self.ui.spin_J5,
            self.ui.spin_J6
        ]

        # Nút tăng (btn_inc_Jn)
        self.buttons_inc = [
            self.ui.btn_inc_J1,
            self.ui.btn_inc_J2,
            self.ui.btn_inc_J3,
            self.ui.btn_inc_J4,
            self.ui.btn_inc_J5,
            self.ui.btn_inc_J6
        ]

        # Nút giảm (btn_des_Jn)
        self.buttons_des = [
            self.ui.btn_des_J1,
            self.ui.btn_des_J2,
            self.ui.btn_des_J3,
            self.ui.btn_des_J4,
            self.ui.btn_des_J5,
            self.ui.btn_des_J6
        ]

        # Gán sự kiện tăng/giảm: dùng step từ slider_step_rot (deg)
        for i in range(6):
            self.buttons_inc[i].clicked.connect(lambda checked, j=i: self.adjust_joint(j, +self.get_step_rot()))
            self.buttons_des[i].clicked.connect(lambda checked, j=i: self.adjust_joint(j, -self.get_step_rot()))

        # Connect / Disconnect button
        self.ui.btn_connect.clicked.connect(self.toggle_connect)

        # Home button
        self.ui.btn_home_small.clicked.connect(self.move_home)

        # If there's a "btn_setting" in UI, connect it to send_all_joints
        if hasattr(self.ui, "btn_setting"):
            self.ui.btn_setting.clicked.connect(self.send_all_joints)

        # Speed slider initial
        if hasattr(self.ui, "slider_speed"):
            self.ui.slider_speed.setMinimum(100)
            self.ui.slider_speed.setMaximum(5000)
            self.ui.slider_speed.setValue(1000)
            self.ui.slider_speed.valueChanged.connect(self.on_speed_changed)
            self.current_speed = int(self.ui.slider_speed.value())
        else:
            self.current_speed = 1000

        # slider_step_rot show value if label present
        if hasattr(self.ui, "slider_step_rot") and hasattr(self.ui, "lbl_val_step_rot"):
            self.ui.slider_step_rot.setMinimum(1)
            self.ui.slider_step_rot.setMaximum(30)
            self.ui.slider_step_rot.setValue(2)
            self.ui.slider_step_rot.valueChanged.connect(lambda v: self.ui.lbl_val_step_rot.setText(str(v)))

        # Khi spinbox thay đổi => cập nhật HTM
        for sb in self.joint_spinboxes:
            sb.valueChanged.connect(self.update_htm_table)

        # Khởi tạo table_htm nếu có
        if hasattr(self.ui, "table_htm"):
            self.ui.table_htm.setRowCount(4)
            self.ui.table_htm.setColumnCount(4)
            for i in range(4):
                for j in range(4):
                    self.ui.table_htm.setItem(i, j, QtWidgets.QTableWidgetItem("0.000000"))

        # Cập nhật label nút connect ban đầu
        self.update_connect_button_label()

        # Update HTM lần đầu
        self.update_htm_table()

    # ---------- Serial helpers ----------
    def update_connect_button_label(self):
        if self.ser and self.ser.is_open:
            self.ui.btn_connect.setText(f"Disconnect ({self.current_port})")
        else:
            ports = list(serial.tools.list_ports.comports())
            if ports:
                # show up to 3 port names
                names = ", ".join([p.device for p in ports][:3])
                self.ui.btn_connect.setText(f"Connect ({names})")
            else:
                self.ui.btn_connect.setText("Connect (no ports)")

    def toggle_connect(self):
        # If connected -> disconnect
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            self.current_port = None
            print("🔌 Disconnected.")
            self.update_connect_button_label()
            return

        # try to find ports
        ports = list(serial.tools.list_ports.comports())
        if not ports:
            QtWidgets.QMessageBox.warning(self, "No COM", "Không tìm thấy cổng COM. Cắm thiết bị vào rồi thử lại.")
            self.update_connect_button_label()
            return

        # try default port first, else first found
        port_to_try = DEFAULT_SERIAL_PORT
        found = False
        for p in ports:
            if p.device.upper() == DEFAULT_SERIAL_PORT.upper():
                port_to_try = p.device
                found = True
                break
        if not found:
            port_to_try = ports[0].device

        try:
            self.ser = serial.Serial(port_to_try, self.baud, timeout=1)
            self.current_port = port_to_try
            print(f"✅ Connected to {port_to_try} @ {self.baud}")
        except Exception as e:
            QtWidgets.QMessageBox.critical(self, "Serial error", f"Không thể mở {port_to_try}:\n{e}")
            self.ser = None
            self.current_port = None
        self.update_connect_button_label()

    def on_speed_changed(self, v):
        self.current_speed = int(v)
        if hasattr(self.ui, "lbl_val_speed"):
            self.ui.lbl_val_speed.setText(str(self.current_speed))
        print(f"Speed = {self.current_speed}")

    # ---------- Servo command ----------
    def send_servo(self, servo_index: int, angle_deg: float, speed: int = None):
        """Send servo command formatted as #<id>P<pulse>T<speed>\r\n"""
        if speed is None:
            speed = self.current_speed
        if not (0 <= servo_index <= 6):
            print("Invalid servo index", servo_index)
            return
        pulse = angle_to_pulse(servo_index, angle_deg)
        cmd = f"#{servo_index+1}P{pulse}T{int(speed)}\r\n"
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(cmd.encode("ascii"))
                print("Gửi:", cmd.strip())
            except Exception as e:
                print("Lỗi khi gửi serial:", e)
        else:
            # Khi chưa nối, in ra để debug
            print("Serial chưa mở — (simulate) Gửi:", cmd.strip())

    # ---------- Joint control ----------
    def get_step_rot(self):
        if hasattr(self.ui, "slider_step_rot"):
            return int(self.ui.slider_step_rot.value())
        return 2

    def adjust_joint(self, joint_index: int, delta_deg: float):
        sb = self.joint_spinboxes[joint_index]
        new_val = sb.value() + delta_deg
        new_val = max(0, min(180, new_val))  # giới hạn 0..180
        sb.setValue(int(new_val))
        self.send_servo(joint_index, new_val, self.current_speed)
        self.update_htm_table()

    def send_all_joints(self):
        for i, sb in enumerate(self.joint_spinboxes):
            self.send_servo(i, sb.value(), self.current_speed)

    def move_home(self):
        for i, sb in enumerate(self.joint_spinboxes):
            sb.setValue(90)
            self.send_servo(i, 90, self.current_speed)
        self.update_htm_table()

    # ---------- Kinematics: DH and HTM ----------
    def dh_transform(self, a: float, alpha: float, d: float, theta: float):
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        return np.array([
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])

    def update_htm_table(self):
        # Lấy góc khớp (deg) -> rad
        theta_deg = [int(sb.value()) for sb in self.joint_spinboxes]
        theta_rad = [np.radians(t) for t in theta_deg]

        # Bảng DH theo L1,L2,L3 (đã chuyển sang mét)
        # Mình dùng ký hiệu: a_i, alpha_i, d_i, theta_i
        # NOTE: thiết lập DH tùy robot — dưới đây là theo doc bạn gửi (3 khớp đầu)
        DH = [
            # a,       alpha,       d,         theta
            [0.0,     np.pi/2,     L1,        theta_rad[0]],   # joint1
            [L2,      0.0,         0.0,       theta_rad[1]],   # joint2
            [L3,      0.0,         0.0,       theta_rad[2]],   # joint3
            # cho khớp 4..6 mình giữ cấu trúc dạng giả định (xoay/wrist)
            [0.0,     np.pi/2,     0.0,       theta_rad[3]],
            [0.0,    -np.pi/2,     0.0,       theta_rad[4]],
            [0.0,     0.0,         0.0,       theta_rad[5]]
        ]

        # Tích liên tiếp
        T = np.eye(4)
        for (a, alpha, d, theta) in DH:
            T = T @ self.dh_transform(a, alpha, d, theta)

        # Hiển thị lên bảng table_htm (4x4) nếu có
        if hasattr(self.ui, "table_htm"):
            for i in range(4):
                for j in range(4):
                    val = float(T[i, j])
                    self.ui.table_htm.setItem(i, j, QtWidgets.QTableWidgetItem(f"{val:.6f}"))

        # (Tùy chọn) in ra để kiểm tra
        print("HTM (end-effector):\n", np.round(T, 6))

# ----------------- Main -----------------
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    win = RobotController()
    win.show()
    sys.exit(app.exec_())
