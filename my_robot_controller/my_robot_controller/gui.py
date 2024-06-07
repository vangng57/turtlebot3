import tkinter as tk
from tkinter import ttk
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from nav_msgs.msg import Odometry
import math

class ROS2GUI(Node):
    def __init__(self):
        super().__init__('ros2_gui')
        self.subscri_ = self.create_subscription(String, 'feedback', self.sub_callback, 10)
        self.odom_sub_ = self.create_subscription(Odometry, '/odom', self.odom_sub_callback, 10)
        self.publish_ = self.create_publisher(Int32, 'cmdToControl', 10)
        self.param_setup()
        self.gui_setup()

    def param_setup(self):
        self.cond1 = True
        self.cond2 = True
        self.cond3 = False
        self.cond4 = True
        self.cond5 = True
        self.cond6 = True
        self.text_data = [
            "Bring up",
            "Send the work area",
            "Delete the work area",
            "Send the path",
            "Delete the path",
            "Start from the first point",
            "Start from the nearest point",
            "Stop following the path",
            "Continue following the path",
            "Quit"
        ]

    def sub_callback(self, msg : String):
        self.feedback_label.config(text=f"Feedback: {msg.data}")

    def odom_sub_callback(self, msg : Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # Convert quaternion to Euler angles to get theta
        orientation_q = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation_q.w * orientation_q.z + orientation_q.x * orientation_q.y)
        cosy_cosp = 1 - 2 * (orientation_q.y * orientation_q.y + orientation_q.z * orientation_q.z)
        theta = math.atan2(siny_cosp, cosy_cosp)

        self.odom_feedback.config(text=f"Position: x={x:.2f}, y={y:.2f}, θ={theta:.2f}")

    def gui_setup(self):
        self.root = tk.Tk()
        self.root.title("Control Panel")
        self.root.protocol("WM_DELETE_WINDOW", self.shutdown_gui)  # Handle window close

        self.root.grid_columnconfigure(0, weight=1, minsize=150)
        self.root.grid_columnconfigure(1, weight=1, minsize=150)
        self.root.grid_columnconfigure(2, weight=1, minsize=150)
        self.root.grid_rowconfigure(0, weight=1, minsize=50)
        self.root.grid_rowconfigure(1, weight=1, minsize=50)
        self.root.grid_rowconfigure(2, weight=1, minsize=50)
        self.root.grid_rowconfigure(3, weight=1, minsize=50)
        self.root.grid_rowconfigure(4, weight=1, minsize=50)

        # Label 1 trở thành nút bấm
        button1 = tk.Button(self.root, text=self.text_data[0], bg="lightblue", width=25, height=2, command=self.button1_callback)
        button1.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        # Label 2 trở thành khung hiển thị tin nhắn từ topic đăng ký
        self.feedback_label = tk.Label(self.root, text="Feedback: Waiting for messages...", width=25, height=2, bg="white", relief="groove", bd=2)
        self.feedback_label.grid(row=0, column=1, columnspan=2, padx=10, pady=10, sticky="nsew")

        # Label 3 trở thành nút bấm
        self.button2_0 = tk.Button(self.root, text=self.text_data[1], bg="lightblue", width=25, height=2, command=self.button2_0_callback)
        self.button2_0.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")
        self.button2_1 = tk.Button(self.root, text=self.text_data[5], bg="lightblue", width=25, height=2, command=self.button2_1_callback)
        self.button2_1.grid(row=1, column=1, padx=10, pady=10, sticky="nsew")

        # Thêm label odom_feedback để hiển thị thông tin từ topic odom
        self.odom_feedback = tk.Label(self.root, text="Position: Waiting for odom...", width=25, height=6, bg="white", relief="groove", bd=2)
        self.odom_feedback.grid(row=1, column=2, rowspan=3, padx=10, pady=10, sticky="nsew")

        self.button3_0 = tk.Button(self.root, text=self.text_data[3], bg="lightblue", width=25, height=2, command=self.button3_0_callback)
        self.button3_0.grid(row=2, column=0, padx=10, pady=10, sticky="nsew")
        self.button3_1 = tk.Button(self.root, text=self.text_data[6], bg="lightblue", width=25, height=2, command=self.button3_1_callback)
        self.button3_1.grid(row=2, column=1, padx=10, pady=10, sticky="nsew")
        self.button4_1 = tk.Button(self.root, text=self.text_data[7], bg="lightblue", width=25, height=2, command=self.button4_1_callback)
        self.button4_1.grid(row=3, column=1, padx=10, pady=10, sticky="nsew")
        self.button4_0 = tk.Button(self.root, text=self.text_data[9], bg="lightblue", width=25, height=2, command=self.shutdown_gui)
        self.button4_0.grid(row=3, column=0, padx=10, pady=10, sticky="nsew")

    def button1_callback(self):
        cmd = Int32()
        cmd.data = 1
        self.publish_.publish(cmd)
        print(cmd.data)
        

    def button2_0_callback(self):
        cmd = Int32()
        if self.button2_0.cget('text') == self.text_data[1]:
            self.button2_0.config(text=self.text_data[2])
            cmd.data = 2
            self.publish_.publish(cmd)
        else:
            self.button2_0.config(text=self.text_data[1])
            cmd.data = 3
            self.publish_.publish(cmd)

        print(cmd.data)

    def button2_1_callback(self):
        cmd = Int32()
        if self.cond3 == True:
            if self.cond1 == True and self.cond2 == True:
                self.button2_1.config(bg="gray")
                self.cond1 = False
                cmd.data = 4
                self.publish_.publish(cmd)
            elif self.cond1 == False and self.cond2 == True:
                self.button2_1.config(bg="lightblue")
                self.cond1 = True
                cmd.data = 5
                self.publish_.publish(cmd)

        print(cmd.data)

    def button3_0_callback(self):
        cmd = Int32()
        if self.button3_0.cget('text') == self.text_data[3]:
            self.button3_0.config(text=self.text_data[4])
            self.cond3 = True
            cmd.data = 6
            self.publish_.publish(cmd)
        else:
            self.button3_0.config(text=self.text_data[3])
            self.cond3 = False
            self.cond1 = True
            self.cond2 = True
            self.button2_1.config(bg="lightblue")
            self.button3_1.config(bg="lightblue")
            cmd.data = 7
            self.publish_.publish(cmd)

    
        print(cmd.data)

    def button3_1_callback(self):
        cmd = Int32()
        if self.cond3 == True:
            if self.cond2 == True and self.cond1 == True:
                self.button3_1.config(bg="gray")
                self.cond2 = False
                cmd.data = 8
                self.publish_.publish(cmd)
            elif self.cond2 == False and self.cond1 == True:
                self.button3_1.config(bg="lightblue")
                self.cond2 = True
                cmd.data = 9
                self.publish_.publish(cmd)

        print(cmd.data)

    def button4_1_callback(self):
        cmd = Int32()
        if self.cond3 == True:
            if self.button4_1.cget('text') == self.text_data[7]:
                self.button4_1.config(text=self.text_data[8])
                cmd.data = 10
                self.publish_.publish(cmd)
            else:
                self.button4_1.config(text=self.text_data[7])
                cmd.data = 11
                self.publish_.publish(cmd)
    
        print(cmd.data)

    def run(self):
        self.root.mainloop()

    def shutdown_gui(self):
        cmd = Int32()
        cmd.data = 12
        self.publish_.publish(cmd)
        print(cmd.data)
        rclpy.shutdown()
        self.root.quit()

def main():
    rclpy.init(args=None)
    gui_publisher = ROS2GUI()
    gui_publisher.run()

if __name__ == '__main__':
    main()