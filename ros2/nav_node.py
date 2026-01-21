#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np

class NavNode(Node):
    def __init__(self):
        super().__init__('nav_node')
        self.sub = self.create_subscription(Float32MultiArray, 'vision', self.vision_cb, 10)
        self.pub = self.create_publisher(Float32MultiArray, 'nav_cmd', 10)

    def vision_cb(self, msg):
        obs = np.array(msg.data)
        direction = obs[:2]
        action = 0.1 * direction / (np.linalg.norm(direction) + 1e-6)
        out = Float32MultiArray()
        out.data = action
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = NavNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
