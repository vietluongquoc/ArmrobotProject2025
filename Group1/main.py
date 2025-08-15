import sys
import serial
import time
import math
import numpy as np
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QTableWidgetItem, QDialog, QVBoxLayout,
    QLabel, QSlider, QPushButton, QHBoxLayout, QSpinBox
)
from PyQt6.QtCore import Qt
from robotui import Ui_MainWindow

SERIAL_PORT = 'COM4'
BAUD_RATE = 115200

class RobotArmController(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # state
        self.servo_angles = [90] * 6          # degrees
        self.step_rotation = 2                # deg (STEP-ROT default like webpage)
        self.step_cart = 0.01                 # meters (STEP-DIS default 1 cm)
        self.speed_level = 1                  # speed-level (1..n) used as gain/iterations

        # wire up joint buttons (use current step_rotation when pressed)
        self.ui.inc1.clicked.connect(lambda: self.move_servo(0, self.step_rotation))
        self.ui.des1.clicked.connect(lambda: self.move_servo(0, -self.step_rotation))
        self.ui.inc2.clicked.connect(lambda: self.move_servo(1, self.step_rotation))
        self.ui.des2.clicked.connect(lambda: self.move_servo(1, -self.step_rotation))
        self.ui.inc3.clicked.connect(lambda: self.move_servo(2, self.step_rotation))
        self.ui.des3.clicked.connect(lambda: self.move_servo(2, -self.step_rotation))
        self.ui.inc4.clicked.connect(lambda: self.move_servo(3, self.step_rotation))
        self.ui.des4.clicked.connect(lambda: self.move_servo(3, -self.step_rotation))
        self.ui.inc5.clicked.connect(lambda: self.move_servo(4, self.step_rotation))
        self.ui.des5.clicked.connect(lambda: self.move_servo(4, -self.step_rotation))
        self.ui.inc6.clicked.connect(lambda: self.move_servo(5, self.step_rotation))
        self.ui.des6.clicked.connect(lambda: self.move_servo(5, -self.step_rotation))

        # HOME and SETTING
        # try both names (btn_home or btn_pos_home depending on your UI)
        if hasattr(self.ui, "btn_home"):
            self.ui.btn_home.clicked.connect(self.reset_all_servos)
        if hasattr(self.ui, "btn_pos_home"):
            self.ui.btn_pos_home.clicked.connect(self.reset_all_servos)
        # setting
        self.ui.btn_setting.clicked.connect(self.open_settings_dialog)

        # Position control buttons (map like web):
        # ↑ Z+  (btn_pos_z_plus), ↓ Z- (btn_pos_z_minus)
        # X+ ↙ (btn_pos_x_plus), X- ↗ (btn_pos_x_minus)
        # O (home) already mapped above
        if hasattr(self.ui, "btn_pos_z_plus"):
            self.ui.btn_pos_z_plus.clicked.connect(lambda: self.move_cartesian(0.0, 0.0,  self.step_cart))
        if hasattr(self.ui, "btn_pos_z_minus"):
            self.ui.btn_pos_z_minus.clicked.connect(lambda: self.move_cartesian(0.0, 0.0, -self.step_cart))
        if hasattr(self.ui, "btn_pos_x_plus"):
            # emulate ↙ X+ as +X in world coords
            self.ui.btn_pos_x_plus.clicked.connect(lambda: self.move_cartesian( self.step_cart, 0.0, 0.0))
        if hasattr(self.ui, "btn_pos_x_minus"):
            # emulate X- ↗ as -X in world coords
            self.ui.btn_pos_x_minus.clicked.connect(lambda: self.move_cartesian(-self.step_cart, 0.0, 0.0))

        # initialize table empty
        for r in range(4):
            for c in range(4):
                self.ui.table_tmatrix.setItem(r, c, QTableWidgetItem(""))

        # serial
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            print("✅ Serial connected")
        except Exception as e:
            print("❌ Serial error:", e)
            self.ser = None

        # reset and show
        self.reset_all_servos()
        self.show_matrix()

    # ---------------- servo / UI ----------------
    def move_servo(self, index, delta_deg):
        """Change one servo by delta_deg (deg)."""
        self.servo_angles[index] = max(0, min(180, self.servo_angles[index] + delta_deg))
        self.send_servo_command(index)
        self.update_joint_display(index)
        self.show_matrix()

    def send_servo_command(self, index):
        angle = self.servo_angles[index]
        pulse = int(500 + (2000 * angle / 180))
        servo_id = index + 1
        cmd = f"#{servo_id}P{pulse}T200\r\n"
        print("TX:", cmd.strip())
        if self.ser:
            try:
                self.ser.write(cmd.encode())
            except Exception as e:
                print("Serial write error:", e)

    def update_joint_display(self, index):
        vs = str(self.servo_angles[index])
        try:
            if index == 0: self.ui.line_j1_val.setText(vs)
            elif index == 1: self.ui.line_j2_val.setText(vs)
            elif index == 2: self.ui.line_j3_val.setText(vs)
            elif index == 3: self.ui.line_j4_val.setText(vs)
            elif index == 4: self.ui.line_j3_val_3.setText(vs)
            elif index == 5: self.ui.line_j4_val_5.setText(vs)
        except Exception:
            pass

    def reset_all_servos(self):
        print("Reset to home (90°)")
        for i in range(6):
            self.servo_angles[i] = 90
            self.send_servo_command(i)
            self.update_joint_display(i)
            time.sleep(0.05)
        self.show_matrix()

    # ---------------- kinematics ----------------
    def dh_transform(self, theta_deg, d, a, alpha_deg):
        theta = math.radians(theta_deg)
        alpha = math.radians(alpha_deg)
        ct = math.cos(theta); st = math.sin(theta)
        ca = math.cos(alpha); sa = math.sin(alpha)
        return np.array([
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [0.0,    sa,     ca,    d],
            [0.0,    0.0,    0.0,  1.0]
        ], dtype=float)

    def forward_kinematics(self, angles=None):
        """Return 4x4 T and position vector (x,y,z)."""
        if angles is None:
            angles = self.servo_angles
        dh_params = [
            [angles[0], 0.100, 0.0,   90],
            [angles[1], 0.0,   0.100, 0],
            [angles[2], 0.0,   0.074, 0],
            [angles[3], 0.013, 0.0,   90],
            [angles[4], 0.0,   0.005, -90],
            [angles[5], 0.0,   0.0,    0]
        ]
        T = np.eye(4)
        for p in dh_params:
            T = T @ self.dh_transform(*p)
        # enforce home target if exact home
        if all(int(a)==90 for a in angles):
            T[0,3] = 0.274
            T[1,3] = 0.0
            T[2,3] = 0.187
        pos = np.array([T[0,3], T[1,3], T[2,3]], dtype=float)
        return T, pos

    def numeric_jacobian(self, eps_deg=0.01):
        """Numerical position Jacobian 3x6 (dp / dtheta_rad)."""
        base_T, p0 = self.forward_kinematics(self.servo_angles)
        J = np.zeros((3,6), dtype=float)
        eps = eps_deg
        for j in range(6):
            pert = list(self.servo_angles)
            pert[j] += eps
            _, pj = self.forward_kinematics(pert)
            dp = pj - p0
            J[:, j] = dp / math.radians(eps)  # dp / rad
        return J

    def damped_least_squares(self, J, delta_pos, lam=0.05):
        """DLS solver returning delta_theta (rad)."""
        JJt = J @ J.T
        A = JJt + (lam**2) * np.eye(3)
        try:
            Ainv = np.linalg.inv(A)
        except np.linalg.LinAlgError:
            Ainv = np.linalg.pinv(A)
        delta_theta = J.T @ (Ainv @ delta_pos)
        return delta_theta

    # ---------------- position control (IK) ----------------
    def move_cartesian(self, dx, dy, dz, max_iter=5):
        """Try to move end-effector by (dx,dy,dz) (meters) using DLS IK.
           This keeps orientation fixed and only changes joint angles to achieve position.
        """
        # small target per call (dx,dy,dz should be small)
        target = np.array([dx, dy, dz], dtype=float)
        # scaling by speed_level (higher => bigger step applied in fewer iterations)
        gain = 1.0 * self.speed_level
        target = target * gain

        for it in range(max_iter):
            T, pos = self.forward_kinematics(self.servo_angles)
            J = self.numeric_jacobian()
            # compute delta_theta (rad)
            delta_theta_rad = self.damped_least_squares(J, target, lam=0.05)
            delta_deg = np.degrees(delta_theta_rad)
            # apply a fraction to avoid overshoot
            apply_frac = 1.0
            for i in range(6):
                self.servo_angles[i] = max(0, min(180, self.servo_angles[i] + apply_frac * delta_deg[i]))
            # send commands
            for i in range(6):
                self.send_servo_command(i)
                self.update_joint_display(i)
            # recompute residual
            _, newpos = self.forward_kinematics(self.servo_angles)
            achieved = newpos - pos
            # if achieved is close to target, stop
            if np.linalg.norm(achieved - target) < 1e-4:
                break
            # otherwise try to correct remaining (set target to remaining)
            target = target - achieved
        # refresh matrix
        self.show_matrix()

    # ---------------- display ----------------
    def show_matrix(self):
        T, pos = self.forward_kinematics(self.servo_angles)
        # display with 3 decimals (like web)
        for r in range(4):
            for c in range(4):
                self.ui.table_tmatrix.setItem(r, c, QTableWidgetItem(f"{T[r,c]:.3f}"))

    # ---------------- settings dialog (like web) ----------------
    def open_settings_dialog(self):
        dialog = QDialog(self)
        dialog.setWindowTitle("Settings")

        v = QVBoxLayout()

        # STEP-DIS (cm)
        h1 = QHBoxLayout()
        lbl_dis = QLabel(f"STEP-DIS (cm): {int(self.step_cart*100)}")
        slider_dis = QSlider(Qt.Orientation.Horizontal)
        slider_dis.setRange(1, 50)  # 1..50 cm
        slider_dis.setValue(int(self.step_cart*100))
        slider_dis.valueChanged.connect(lambda vv: (
            setattr(self, "step_cart", vv / 100.0),
            lbl_dis.setText(f"STEP-DIS (cm): {vv}")
        ))
        h1.addWidget(lbl_dis)
        h1.addWidget(slider_dis)
        v.addLayout(h1)

        # STEP-ROT (deg)
        h2 = QHBoxLayout()
        lbl_rot = QLabel(f"STEP-ROT (deg): {self.step_rotation}")
        slider_rot = QSlider(Qt.Orientation.Horizontal)
        slider_rot.setRange(1, 30)
        slider_rot.setValue(self.step_rotation)
        slider_rot.valueChanged.connect(lambda vv: (
            setattr(self, "step_rotation", vv),
            lbl_rot.setText(f"STEP-ROT (deg): {vv}")
        ))
        h2.addWidget(lbl_rot)
        h2.addWidget(slider_rot)
        v.addLayout(h2)

        # SPEED-LEVEL
        h3 = QHBoxLayout()
        lbl_spd = QLabel(f"SPEED-LEVEL: {self.speed_level}")
        slider_spd = QSlider(Qt.Orientation.Horizontal)
        slider_spd.setRange(1, 5)
        slider_spd.setValue(self.speed_level)
        slider_spd.valueChanged.connect(lambda vv: (
            setattr(self, "speed_level", vv),
            lbl_spd.setText(f"SPEED-LEVEL: {vv}")
        ))
        h3.addWidget(lbl_spd)
        h3.addWidget(slider_spd)
        v.addLayout(h3)

        # Set / Close buttons
        btn_set = QPushButton("SET")
        btn_set.clicked.connect(lambda: (self.show_matrix(), dialog.close()))
        btn_close = QPushButton("Close")
        btn_close.clicked.connect(dialog.close)
        v.addWidget(btn_set)
        v.addWidget(btn_close)

        dialog.setLayout(v)
        dialog.exec()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RobotArmController()
    window.show()
    sys.exit(app.exec())