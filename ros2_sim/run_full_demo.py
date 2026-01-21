#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import matplotlib.pyplot as plt
from sim_env import SimEnv

class LiveRoboNullDemo(Node):
    def __init__(self):
        super().__init__('live_robonull_demo')

        # подписка на финальные действия от meta_null_node
        self.sub_final = self.create_subscription(
            Float32MultiArray,
            'final_cmd',
            self.final_cmd_cb,
            10
        )

        # симулятор
        self.env = SimEnv()
        self.action = np.zeros(2)

        # логи для графиков
        self.phi_meta_log = []
        self.collisions_log = []

        # графики
        plt.ion()
        self.fig, self.axs = plt.subplots(2,1, figsize=(7,6))
        self.axs[0].set_title('Live RoboNull Demo')
        self.axs[0].set_ylabel('Φ_meta')
        self.axs[1].set_ylabel('Collisions')
        self.axs[1].set_xlabel('Time Step')

        # таймер ROS2
        self.timer = self.create_timer(0.05, self.update_sim)

    def final_cmd_cb(self, msg):
        self.action = np.array(msg.data, dtype=np.float32)

    def update_sim(self):
        # делаем шаг в симуляции
        obs, done = self.env.step(self.action)

        # Φ_meta proxy: квадрат нормы действия
        phi_meta = np.linalg.norm(self.action)**2
        self.phi_meta_log.append(phi_meta)
        self.collisions_log.append(self.env.collisions)

        # обновляем графики
        self.axs[0].cla()
        self.axs[0].plot(self.phi_meta_log, color='blue')
        self.axs[0].set_ylabel('Φ_meta')

        self.axs[1].cla()
        self.axs[1].plot(self.collisions_log, color='red')
        self.axs[1].set_ylabel('Collisions')
        self.axs[1].set_xlabel('Time Step')

        plt.pause(0.001)

        # конец симуляции
        if done:
            print(f"Simulation finished. Total collisions: {self.env.collisions}")
            plt.ioff()
            plt.show()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = LiveRoboNullDemo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
