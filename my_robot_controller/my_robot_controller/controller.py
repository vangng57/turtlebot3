import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String
import tkinter as tk
import threading

class GUIPublisher(Node):
    def __init__(self):
        super().__init__('gui_publisher')
        self.publisher_ = self.create_publisher(Int32, 'cmdToControl', 10)
        self.subscription = self.create_subscription(String, 'feedback', self.feedback_callback, 10)
        self.gui_setup()
        # Khởi động các luồng độc lập
        self.start_threads()

    def publish_number(self, number):
        msg = Int32()
        msg.data = number
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {number}')
        if number == 7:  # If the 'quit the program' button is pressed
            self.shutdown_gui()  # Call shutdown function

    def feedback_callback(self, msg):
        # An toàn cập nhật giao diện người dùng từ luồng khác
        self.root.after(0, self.update_feedback_label, msg.data)

    def update_feedback_label(self, data):
        self.feedback_label.config(text=f"Feedback: {data}")

    def gui_setup(self):
        self.root = tk.Tk()
        self.root.title("Control Panel")
        self.root.protocol("WM_DELETE_WINDOW", self.shutdown_gui)  # Handle window close
        self.feedback_label = tk.Label(self.root, text="Feedback: Waiting for messages...", height=2)
        self.feedback_label.pack(pady=10, padx=10)

        button_info = [
            ("send the Path", 1),
            ("start from the first point", 2),
            ("start from the nearest point", 3),
            ("delete the path", 4),
            ("stop following the path", 5),
            ("continue following the path", 6),
            ("quit the program", 7),
        ]
        max_width = max(len(text) for text, _ in button_info)
        for text, number in button_info:
            button = tk.Button(self.root, text=text, width=max_width, command=lambda num=number: self.publish_number(num))
            button.pack(pady=5, padx=10, ipadx=10, ipady=5)

    def start_threads(self):
        # Luồng cho việc nhận và xử lý tin nhắn ROS
        ros_thread = threading.Thread(target=self.ros_spin)
        ros_thread.start()

    def ros_spin(self):
        rclpy.spin(self)

    def run(self):
        # Chạy giao diện người dùng trong luồng chính
        self.root.mainloop()

    def shutdown_gui(self):
        rclpy.shutdown()
        self.root.quit()

def main(args=None):
    rclpy.init(args=args)
    gui_publisher = GUIPublisher()
    gui_publisher.run()

if __name__ == '__main__':
    main()