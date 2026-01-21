from env import SimpleEnv
from scenarios import vision, nav_policy, grasp_policy
from meta_null import meta_null
import numpy as np

def run(mode='baseline', steps=100):
    env = SimpleEnv()
    obs = env.reset()

    phi_meta_log = []

    for _ in range(steps):
        psi_v = vision(obs)

        psi_nav = nav_policy(psi_v)
        psi_grasp = grasp_policy(psi_v)

        psi_stack = np.stack([psi_nav, psi_grasp])

        if mode == 'robonull':
            meta = meta_null(psi_stack)
            action = psi_nav + psi_grasp + meta
            phi_meta = ((psi_stack - psi_stack.mean(0))**2).mean()
            phi_meta_log.append(phi_meta)
        else:
            action = psi_nav + psi_grasp

        obs, _, done, _ = env.step(action)

        if done:
            break

    return {
        'collisions': env.collisions,
        'steps': env.t,
        'phi_meta': np.mean(phi_meta_log) if phi_meta_log else None
    }


if __name__ == '__main__':
    print("Baseline:", run('baseline'))
    print("RoboNull:", run('robonull'))
