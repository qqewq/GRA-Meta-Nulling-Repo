#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from sim_env import SimEnv

class RoboNullDemoGIF(Node):
    def __init__(self):
        super().__init__('robonull_demo_gif')
        self.sub = self.create_subscription(Float32MultiArray, 'final_cmd', self.cmd_cb, 10)
        self.env = SimEnv()
        self.action = np.zeros(2)

        # Logs
        self.phi_meta_log = []
        self.collisions_log = []
        self.positions = []

        # Plot setup
        self.fig, self.axs = plt.subplots(2,1, figsize=(7,6))
        self.axs[0].set_title('Live RoboNull Demo')
        self.axs[0].set_ylabel('Φ_meta')
        self.axs[1].set_ylabel('Collisions')
        self.axs[1].set_xlabel('Time Step')

        # Animation writer
        self.frames = []

        # Timer ROS2
        self.timer = self.create_timer(0.05, self.update)

    def cmd_cb(self, msg):
        self.action = np.array(msg.data, dtype=np.float32)

    def update(self):
        obs, done = self.env.step(self.action)

        # Logs
        phi_meta = np.linalg.norm(self.action)**2
        self.phi_meta_log.append(phi_meta)
        self.collisions_log.append(self.env.collisions)
        self.positions.append(self.env.robot.copy())

        # Live plotting
        self.axs[0].cla()
        self.axs[0].plot(self.phi_meta_log, color='blue')
        self.axs[0].set_ylabel('Φ_meta')

        self.axs[1].cla()
        self.axs[1].plot(self.collisions_log, color='red')
        self.axs[1].set_ylabel('Collisions')
        self.axs[1].set_xlabel('Time Step')

        plt.pause(0.001)

        # Store frame for GIF
        self.frames.append([self.axs[0].lines[0], self.axs[1].lines[0]])

        if done:
            print(f"Simulation finished. Total collisions: {self.env.collisions}")
            plt.ioff()
            self.save_gif()
            rclpy.shutdown()

    def save_gif(self):
        print("Saving GIF...")
        fig_anim, axs_anim = plt.subplots(2,1, figsize=(7,6))
        axs_anim[0].set_ylabel('Φ_meta')
        axs_anim[1].set_ylabel('Collisions')
        axs_anim[1].set_xlabel('Time Step')

        def update_frame(i):
            axs_anim[0].cla()
            axs_anim[0].plot(self.phi_meta_log[:i+1], color='blue')
            axs_anim[0].set_ylabel('Φ_meta')
            axs_anim[1].cla()
            axs_anim[1].plot(self.collisions_log[:i+1], color='red')
            axs_anim[1].set_ylabel('Collisions')
            axs_anim[1].set_xlabel('Time Step')

        anim = FuncAnimation(fig_anim, update_frame, frames=len(self.phi_meta_log), interval=50)
        anim.save('robonull_demo.gif', writer=PillowWriter(fps=20))
        print("GIF saved as robonull_demo.gif")

def main(args=None):
    rclpy.init(args=args)
    node = RoboNullDemoGIF()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
