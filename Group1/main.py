import sys
import serial
from PyQt5 import QtWidgets, QtCore
from ui_robot_control import Ui_MainWindow

try:
    arduino = serial.Serial('COM3', 9600, timeout=0.1)
except Exception as e:
    arduino = None
    print("Không kết nối Arduino:", e)

class RobotControlApp(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.setup_buttons()
        self.start_serial_read()
        self.update_tmatrix([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])

    def setup_buttons(self):
        self.ui.btn_j1_inc.clicked.connect(lambda: self.send_cmd("J1+"))
        self.ui.btn_j1_des.clicked.connect(lambda: self.send_cmd("J1-"))
        self.ui.btn_j2_inc.clicked.connect(lambda: self.send_cmd("J2+"))
        self.ui.btn_j2_des.clicked.connect(lambda: self.send_cmd("J2-"))
        self.ui.btn_j3_inc.clicked.connect(lambda: self.send_cmd("J3+"))
        self.ui.btn_j3_des.clicked.connect(lambda: self.send_cmd("J3-"))
        self.ui.btn_j4_inc.clicked.connect(lambda: self.send_cmd("J4+"))
        self.ui.btn_j4_des.clicked.connect(lambda: self.send_cmd("J4-"))
        self.ui.btn_gripper_on.clicked.connect(lambda: self.send_cmd("GRIPPER=ON"))
        self.ui.btn_pos_x_plus.clicked.connect(lambda: self.send_cmd("POS+X"))
        self.ui.btn_pos_x_minus.clicked.connect(lambda: self.send_cmd("POS-X"))
        self.ui.btn_pos_z_plus.clicked.connect(lambda: self.send_cmd("POS+Z"))
        self.ui.btn_pos_z_minus.clicked.connect(lambda: self.send_cmd("POS-Z"))
        self.ui.btn_pos_home.clicked.connect(lambda: self.send_cmd("POS=HOME"))
        self.ui.btn_home.clicked.connect(lambda: self.send_cmd("CMD=HOME"))
        self.ui.btn_setting.clicked.connect(lambda: self.send_cmd("CMD=SETTING"))

    def send_cmd(self, cmd):
        if arduino and arduino.is_open:
            arduino.write((cmd + "\n").encode())
            print("Sent:", cmd)
        else:
            print("Arduino chưa kết nối!")

    def start_serial_read(self):
        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.read_serial)
        timer.start(100)

    def read_serial(self):
        if arduino and arduino.in_waiting:
            line = arduino.readline().decode(errors='ignore').strip()
            print("Arduino:", line)
            if line.startswith("TMATRIX:"):
                raw = line.split("TMATRIX:")[1]
                rows = raw.split(";")
                mat = [list(map(int, r.split(","))) for r in rows]
                self.update_tmatrix(mat)

    def update_tmatrix(self, mat):
        for i in range(4):
            for j in range(4):
                item = QtWidgets.QTableWidgetItem(str(mat[i][j]))
                item.setTextAlignment(QtCore.Qt.AlignCenter)
                self.ui.table_tmatrix.setItem(i, j, item)

if __name__ == "_main_":
    app = QtWidgets.QApplication(sys.argv)
    window = RobotControlApp()
    window.show()
    sys.exit(app.exec_())