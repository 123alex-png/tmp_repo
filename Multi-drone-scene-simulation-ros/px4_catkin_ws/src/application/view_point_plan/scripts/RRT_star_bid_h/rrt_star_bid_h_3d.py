# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
import numpy as np

from RRT.geometry import *
from RRT.plotting import Plot
from RRT.search_space import SearchSpace
from rrt_star_bid_h import RRTStarBidirectionalHeuristic

X_dimensions = np.array([(0, 100), (0, 100),
                         (0, 100)])  # dimensions of Search Space

Obstacles = np.array([(10, 10, 30, 85, 50, 35), (30, 20, 0, 35, 25, 30),
                      (60, 20, 0, 65, 25, 30), (30, 35, 0, 35, 40, 30),
                      (60, 35, 0, 65, 40, 30)])
x_init = (20, 30, 0)  # starting location
x_goal = (40, 30, 50)  # goal location

Q = np.array([(8, 4)])  # length of tree edges
r = 1  # length of smallest edge to check for intersection with obstacles
max_samples = 1024  # max number of samples to take before timing out
rewire_count = 32  # optional, number of nearby branches to rewire
prc = 0.01  # probability of checking for a connection to goal

# create Search Space
X = SearchSpace(X_dimensions, Obstacles)

# create rrt_search
rrt = RRTStarBidirectionalHeuristic(X, Q, x_init, x_goal, max_samples, r, prc,
                                    rewire_count)
path = rrt.rrt_star_bid_h()

# plot
plot = Plot("rrt_star_bid_h_3d")
plot.plot_tree(X, rrt.trees)
if path is not None:
    plot.plot_path(X, path)
plot.plot_obstacles(X, Obstacles)
plot.plot_start(X, x_init)
plot.plot_goal(X, x_goal)
plot.draw(auto_open=True)
