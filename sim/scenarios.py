import numpy as np

def vision(obs):
    # perceives obstacle & target
    return np.concatenate([
        obs['target'] - obs['robot'],
        obs['obstacle'] - obs['robot']
    ])

def nav_policy(psi_v):
    # wants to go straight to target
    direction = psi_v[:2]
    return 0.1 * direction / (np.linalg.norm(direction) + 1e-6)

def grasp_policy(psi_v):
    # tries to slow down when obstacle near (conflict)
    obstacle_vec = psi_v[2:]
    dist = np.linalg.norm(obstacle_vec)
    if dist < 2.0:
        return -0.05 * obstacle_vec
    return np.zeros(2)
