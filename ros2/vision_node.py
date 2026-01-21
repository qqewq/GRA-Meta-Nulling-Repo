#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.pub = self.create_publisher(Float32MultiArray, 'vision', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        # dummy perception: target & obstacle positions
        msg = Float32MultiArray()
        msg.data = np.array([10.0, 0.0, 5.0, 0.0], dtype=np.float32)  # target, obstacle
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
