#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np

class GraspNode(Node):
    def __init__(self):
        super().__init__('grasp_node')
        self.sub = self.create_subscription(Float32MultiArray, 'vision', self.vision_cb, 10)
        self.pub = self.create_publisher(Float32MultiArray, 'grasp_cmd', 10)

    def vision_cb(self, msg):
        obs = np.array(msg.data)
        obstacle_vec = obs[2:]
        action = -0.05 * obstacle_vec if np.linalg.norm(obstacle_vec) < 2 else np.zeros(2)
        out = Float32MultiArray()
        out.data = action
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = GraspNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
