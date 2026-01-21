#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
from sim_env import SimEnv
import matplotlib.pyplot as plt

class ROS2Sim(Node):
    def __init__(self):
        super().__init__('ros2_sim')
        self.sub = self.create_subscription(Float32MultiArray, 'final_cmd', self.cmd_cb, 10)
        self.env = SimEnv()
        self.action = np.zeros(2)
        self.phi_meta_log = []
        self.collisions_log = []
        plt.ion()
        self.fig, self.axs = plt.subplots(2,1, figsize=(6,6))
        self.timer = self.create_timer(0.05, self.update)

    def cmd_cb(self, msg):
        self.action = np.array(msg.data)

    def update(self):
        obs, done = self.env.step(self.action)
        self.phi_meta_log.append(np.linalg.norm(self.action)**2)  # simple proxy Φ_meta
        self.collisions_log.append(self.env.collisions)

        # live plots
        self.axs[0].cla()
        self.axs[0].plot(self.phi_meta_log)
        self.axs[0].set_ylabel('Φ_meta')

        self.axs[1].cla()
        self.axs[1].plot(self.collisions_log)
        self.axs[1].set_ylabel('Collisions')

        plt.pause(0.001)

        if done:
            print(f"Simulation finished. Collisions: {self.env.collisions}")
            plt.ioff()
            plt.show()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ROS2Sim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
