#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from sim_env import SimEnv

def run_baseline(env, steps=100):
    """Simple baseline: nav + grasp actions without RoboNull coordination"""
    phi_meta_log = []
    collisions_log = []
    env.reset()
    for t in range(steps):
        # Baseline: independent actions
        nav_action = 0.1 * (env.target - env.robot) / (np.linalg.norm(env.target - env.robot) + 1e-6)
        grasp_action = np.array([0.0, 0.0])  # no grasp correct_
