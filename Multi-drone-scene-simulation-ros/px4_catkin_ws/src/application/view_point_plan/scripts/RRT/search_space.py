# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.

import numpy as np
from rtree import index

from RRT.geometry import es_points_along_line
from RRT.obstacle_generation import obstacle_generator


class SearchSpace(object):

    def __init__(self, dimension_lengths, Obstacles=None):
        if len(dimension_lengths) < 3:  # 必须为三个维度 xyz
            raise Exception("Must have at least 3 dimensions")
        self.dimensions = len(dimension_lengths)

        if any(len(i) != 2 for i in
               dimension_lengths):  # 每个维度必须是 两个量 （start，end） =》 （0，100）
            raise Exception("Dimensions can only have a start and end")
        if any(i[0] >= i[1] for i in dimension_lengths):  # 第一个数小于第二个数
            raise Exception("Dimension start must be less than dimension end")
        self.dimension_lengths = dimension_lengths
        p = index.Property()
        p.dimension = self.dimensions
        if Obstacles is None:
            self.obs = index.Index(interleaved=True, properties=p)
        else:
            # r-tree representation of obstacles
            # print(len(dimension_lengths))
            if any(len(o) / 2 != len(dimension_lengths) for o in Obstacles):
                raise Exception("Obstacle has incorrect dimension definition")
            if any(o[i] >= o[int(i + len(o) / 2)] for o in Obstacles
                   for i in range(int(len(o) / 2))):
                raise Exception(
                    "Obstacle start must be less than obstacle end")
            self.obs = index.Index(obstacle_generator(Obstacles),
                                   interleaved=True,
                                   properties=p)

    def obstacle_free(self, x):
        """
        Check if a location resides inside of an obstacle
        :param x: location to check
        :return: True if not inside an obstacle, False otherwise
        """
        return self.obs.count(x) == 0

    def sample_free(self):
        """
        Sample a location within X_free
        :return: random location within X_free
        """
        while True:  # sample until not inside of an obstacle
            x = self.sample()
            if self.obstacle_free(x):
                return x

    def collision_free(self, start, end, r):
        """
        Check if a line segment intersects an obstacle
        :param start: starting point of line
        :param end: ending point of line
        :param r: resolution of points to sample along edge when checking for collisions
        :return: True if line segment does not intersect an obstacle, False otherwise
        """
        points = es_points_along_line(start, end, r)
        coll_free = all(map(self.obstacle_free, points))
        return coll_free

    def sample(self):
        """
        Return a random location within X
        :return: random location within X (not necessarily X_free)
        """
        x = np.random.uniform(self.dimension_lengths[:, 0],
                              self.dimension_lengths[:, 1])
        return tuple(x)
