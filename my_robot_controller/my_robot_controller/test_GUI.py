import tkinter as tk
import subprocess

class App:
    def __init__(self, root):
        self.root = root
        self.root.title("ROS2 Node Launcher with Stateful Buttons")

        # Nút để khởi chạy turtlesim_node trong terminal mới
        self.turtlesim_button = tk.Button(root, text="Launch Turtlesim", command=self.toggle_turtlesim)
        self.turtlesim_button.pack(pady=10)
        self.turtlesim_process = None

        # Nút để khởi chạy teleop_twist_keyboard trong terminal mới
        self.teleop_button = tk.Button(root, text="Launch Teleop", command=self.toggle_teleop)
        self.teleop_button.pack(pady=10)
        self.teleop_process = None

    def toggle_turtlesim(self):
        if self.turtlesim_process and self.turtlesim_process.poll() is None:
            self.turtlesim_process.terminate()
            self.turtlesim_process.wait()  # Chờ tiến trình kết thúc
            self.turtlesim_process = None
            self.turtlesim_button.config(bg='SystemButtonFace', text="Launch Turtlesim")
        else:
            self.turtlesim_process = subprocess.Popen(["gnome-terminal", "--", "ros2", "run", "turtlesim", "turtlesim_node"])
            self.turtlesim_button.config(bg='green', text="Stop Turtlesim")

    def toggle_teleop(self):
        if self.teleop_process and self.teleop_process.poll() is None:
            self.teleop_process.terminate()
            self.teleop_process.wait()  # Chờ tiến trình kết thúc
            self.teleop_process = None
            self.teleop_button.config(bg='SystemButtonFace', text="Launch Teleop")
        else:
            self.teleop_process = subprocess.Popen(["gnome-terminal", "--", "ros2", "run", "teleop_twist_keyboard", "teleop_twist_keyboard"])
            self.teleop_button.config(bg='green', text="Stop Teleop")

def main():
    root = tk.Tk()
    app = App(root)
    root.mainloop()

if __name__ == "__main__":
    main()