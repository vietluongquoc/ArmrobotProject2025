import tkinter as tk
from tkinter import ttk

# Hàm xử lý (ví dụ: in ra console khi bấm nút)
def handle_button(label):
    print(f"Button pressed: {label}")

# Dữ liệu ma trận
matrix = [
    [0.985, 0.167, -0.007, 0.265],
    [-0.168, 0.985, 0.001, -0.046],
    [0.006, 0.000, 0.999, 0.136],
    [0.000, 0.000, 0.000, 1.000],
]

# Tạo cửa sổ chính
root = tk.Tk()
root.title("Robot Control UI")
root.geometry("600x700")
root.configure(bg="white")

# ========== POSITION CONTROL ==========
pos_label = tk.Label(root, text="POSITION CONTROL", font=("Arial", 12, "bold"), bg="white")
pos_label.pack(pady=10)

pos_frame = tk.Frame(root, bg="white")
pos_frame.pack()

tk.Button(pos_frame, text="+Z", bg="lightgreen", command=lambda: handle_button("+Z"), width=6).grid(row=0, column=1)
tk.Button(pos_frame, text="+X", bg="lightgreen", command=lambda: handle_button("+X"), width=6).grid(row=1, column=0)
tk.Button(pos_frame, text="O", bg="lightgreen", command=lambda: handle_button("O"), width=6).grid(row=1, column=1)
tk.Button(pos_frame, text="-X", bg="lightgreen", command=lambda: handle_button("-X"), width=6).grid(row=1, column=2)
tk.Button(pos_frame, text="-Z", bg="lightgreen", command=lambda: handle_button("-Z"), width=6).grid(row=2, column=1)

# ========== JOIN CONTROL ==========
tk.Label(root, text="JOIN CONTROL", font=("Arial", 12, "bold"), bg="white").pack(pady=10)
for i in range(4):
    joint_frame = tk.Frame(root, bg="white")
    joint_frame.pack()
    tk.Label(joint_frame, text=f"JOIN {i+1}", width=8, bg="white").pack(side=tk.LEFT)
    tk.Label(joint_frame, text=str(90), width=5, bg="white").pack(side=tk.LEFT)  # giá trị mẫu
    tk.Button(joint_frame, text="INC ↑", bg="lightgreen", width=6,
              command=lambda i=i: handle_button(f"INC {i+1}")).pack(side=tk.LEFT)
    tk.Button(joint_frame, text="DES ↓", bg="lightgreen", width=6,
              command=lambda i=i: handle_button(f"DES {i+1}")).pack(side=tk.LEFT)

# ========== GRIPPER ==========
tk.Label(root, text="GRIPPER", font=("Arial", 12, "bold"), bg="white").pack(pady=10)
tk.Button(root, text="ON", bg="lightgreen", width=10, command=lambda: handle_button("GRIPPER ON")).pack()

# ========== HOME & SETTING ==========
bottom_frame = tk.Frame(root, bg="white")
bottom_frame.pack(pady=10)
tk.Button(bottom_frame, text="HOME", bg="lightgreen", width=10, command=lambda: handle_button("HOME")).pack(side=tk.LEFT, padx=5)
tk.Button(bottom_frame, text="SETTING", bg="lightgreen", width=10, command=lambda: handle_button("SETTING")).pack(side=tk.LEFT, padx=5)

# ========== HOMOGENEOUS MATRIX ==========
tk.Label(root, text="HOMOGENEOUS TRANSFORMATION MATRICES", font=("Arial", 12, "bold"), bg="white").pack(pady=10)

table_frame = tk.Frame(root, bg="white")
table_frame.pack()

for i in range(4):
    for j in range(4):
        entry = tk.Entry(table_frame, width=10, justify="center")
        entry.grid(row=i, column=j, padx=2, pady=2)
        entry.insert(0, str(matrix[i][j]))
# ========== CHẠY GIAO DIỆN ==========
root.mainloop()