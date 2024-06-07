import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovariance
from std_msgs.msg import Header

class PlanPublisherNode(Node):

    def __init__(self):
        super().__init__('plan_publisher')
        self.publisher = self.create_publisher(Path, 'plan', 10)
        self.amcl_pose_sub = self.create_subscription(
            '/amcl_pose', PoseWithCovariance, self.amcl_pose_callback, 10)

    def amcl_pose_callback(self, msg):
        if not msg.pose.covariance:
            return

        path = Path()
        path.header.stamp = msg.header.stamp
        path.header.frame_id = msg.header.frame_id

        # Sử dụng pose từ amcl_pose làm điểm đầu
        pose1 = PoseStamped()
        pose1.header = msg.header
        pose1.pose = msg.pose
        path.poses.append(pose1)

        # Tạo các điểm khác (nếu cần)
        # ...

        self.publisher.publish(path)
        self.get_logger().info('Published path: %s' % str(path))
        self.amcl_pose_sub.unregister()  # Hủy đăng ký subscriber sau khi publish

def main(args=None):
    rclpy.init(args=args)
    node = PlanPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
