#!/usr/bin/env python3
import matplotlib.pyplot as plt
import numpy as np
from sim_env import SimEnv

class LivePlotter:
    def __init__(self):
        self.env = SimEnv()
        self.phi_meta_log = []
        self.collisions_log = []

        plt.ion()
        self.fig, self.axs = plt.subplots(2,1, figsize=(6,6))
        self.axs[0].set_title('Live RoboNull Metrics')
        self.axs[0].set_ylabel('Φ_meta')
        self.axs[1].set_ylabel('Collisions')
        self.axs[1].set_xlabel('Time Step')

    def step(self, action):
        obs, done = self.env.step(action)
        # proxy for Φ_meta: squared norm of combined action
        phi_meta = np.linalg.norm(action)**2
        self.phi_meta_log.append(phi_meta)
        self.collisions_log.append(self.env.collisions)
        return done

    def update_plot(self):
        self.axs[0].cla()
        self.axs[0].plot(self.phi_meta_log, color='blue')
        self.axs[0].set_ylabel('Φ_meta')

        self.axs[1].cla()
        self.axs[1].plot(self.collisions_log, color='red')
        self.axs[1].set_ylabel('Collisions')
        self.axs[1].set_xlabel('Time Step')

        plt.pause(0.001)

def run_demo(action_sequence):
    plotter = LivePlotter()
    for action in action_sequence:
        done = plotter.step(action)
        plotter.update_plot()
        if done:
            break
    plt.ioff()
    plt.show()
    print(f"Simulation finished. Total collisions: {plotter.env.collisions}")

if __name__ == "__main__":
    # example demo sequence: straight line + some random noise
    actions = [np.array([0.1,0.0]) + 0.02*np.random.randn(2) for _ in range(100)]
    run_demo(actions)
