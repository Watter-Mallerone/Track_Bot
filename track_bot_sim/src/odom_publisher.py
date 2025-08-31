#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import math, time

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('fake_odom_pub')
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.t0 = time.time()

    def timer_callback(self):
        t = time.time() - self.t0
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # 원형 경로로 움직이는 예시
        odom.pose.pose.position.x = math.cos(t)
        odom.pose.pose.position.y = math.sin(t)

        self.pub.publish(odom)

rclpy.init()
node = OdomPublisher()
rclpy.spin(node)

