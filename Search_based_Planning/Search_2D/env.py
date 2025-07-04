"""
Env 2D
@author: huiming zhou
"""


class Env:
    def __init__(self):
        # Initialize map size
        self.x_range = 51   # size of width
        self.y_range = 31   # size of height

        # Initialize neighborhood
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        
        # Initialize obstacles
        self.obs = self.obs_map()

    def update_obs(self, obs):
        self.obs = obs

    def obs_map(self):
        """
        Initialize obstacles' positions
        :return: map of obstacles
        """

        x = self.x_range
        y = self.y_range
        obs = set()
        # Initialize borders
        for i in range(x):  # upper border
            obs.add((i, 0))
        
        for i in range(x):  # bottom border
            obs.add((i, y - 1))

        for i in range(y):  # left border
            obs.add((0, i))

        for i in range(y):  # right border
            obs.add((x - 1, i))

        # Initialize obstacles
        for i in range(10, 21):
            obs.add((i, 15))
        for i in range(15):
            obs.add((20, i))

        for i in range(15, 30):
            obs.add((30, i))
        for i in range(16):
            obs.add((40, i))

        return obs
