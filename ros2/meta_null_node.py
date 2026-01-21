#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np

class MetaNullNode(Node):
    def __init__(self):
        super().__init__('meta_null_node')
        self.sub_nav = self.create_subscription(Float32MultiArray, 'nav_cmd', self.nav_cb, 10)
        self.sub_grasp = self.create_subscription(Float32MultiArray, 'grasp_cmd', self.grasp_cb, 10)
        self.pub = self.create_publisher(Float32MultiArray, 'final_cmd', 10)
        self.nav = None
        self.grasp = None
        self.alpha = 0.3

    def nav_cb(self, msg):
        self.nav = np.array(msg.data)
        self.publish_final()

    def grasp_cb(self, msg):
        self.grasp = np.array(msg.data)
        self.publish_final()

    def publish_final(self):
        if self.nav is None or self.grasp is None:
            return
        stack = np.stack([self.nav, self.grasp])
        center = stack.mean(0)
        meta_corr = self.alpha * (center - stack).mean(0)
        final_action = self.nav + self.grasp + meta_corr
        out = Float32MultiArray()
        out.data = final_action
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = MetaNullNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
