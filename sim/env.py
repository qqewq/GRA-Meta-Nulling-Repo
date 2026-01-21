import numpy as np

class SimpleEnv:
    def __init__(self):
        self.reset()

    def reset(self):
        self.robot = np.array([0.0, 0.0])
        self.target = np.array([10.0, 0.0])
        self.obstacle = np.array([5.0, 0.0])
        self.t = 0
        self.done = False
        self.collisions = 0
        return self._obs()

    def _obs(self):
        return {
            'robot': self.robot.copy(),
            'target': self.target.copy(),
            'obstacle': self.obstacle.copy(),
            'time': self.t
        }

    def step(self, action):
        if self.done:
            return self._obs(), 0, True, {}

        self.robot += action
        self.t += 1

        # obstacle moves into path
        if self.t == 20:
            self.obstacle += np.array([0.0, 1.0])

        if np.linalg.norm(self.robot - self.obstacle) < 0.5:
            self.collisions += 1

        if np.linalg.norm(self.robot - self.target) < 0.5:
            self.done = True

        return self._obs(), -self.collisions, self.done, {}
