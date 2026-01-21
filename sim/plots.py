import matplotlib.pyplot as plt
import numpy as np
from run_sim import run

def collect(mode, runs=30):
    collisions = []
    for _ in range(runs):
        r = run(mode)
        collisions.append(r['collisions'])
    return np.array(collisions)

baseline = collect('baseline')
robonull = collect('robonull')

plt.figure()
plt.hist(baseline, alpha=0.7)
plt.hist(robonull, alpha=0.7)
plt.xlabel("Collisions per episode")
plt.ylabel("Frequency")
plt.title("Baseline vs RoboNull")
plt.show()
