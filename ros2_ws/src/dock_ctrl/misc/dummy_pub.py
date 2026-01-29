#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time

class DummyVisionPosePublisher(Node):
    def __init__(self):
        super().__init__('dummy_vision_pose_pub')
        self.publisher_ = self.create_publisher(PoseStamped, '/mavros/vision_pose/pose', 10)
        timer_period = 1.0 / 20.0  # 20 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info("Publishing dummy vision pose data to /mavros/vision_pose/pose at 20 Hz")

    def timer_callback(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()  # ✅ valid ROS timestamp
        msg.header.frame_id = 'map'

        # Dummy position and orientation
        msg.pose.position.x = 2.2
        msg.pose.position.y = 0.6
        msg.pose.position.z = 1.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = DummyVisionPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
