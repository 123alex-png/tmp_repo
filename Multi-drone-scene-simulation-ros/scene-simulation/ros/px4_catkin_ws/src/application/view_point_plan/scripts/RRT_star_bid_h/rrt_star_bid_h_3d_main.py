# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.

from RRT.geometry import *
from RRT.plotting import Plot
from RRT.search_space import SearchSpace
from RRT_star_bid_h.rrt_star_bid_h import RRTStarBidirectionalHeuristic


def node_is_in_Obstacles(node, Obstacles, r):
    for i, obstacle in enumerate(Obstacles):
        if node[0] >= (obstacle[0] - r) and node[0] <= (obstacle[3] + r) and node[1] >= (obstacle[1] - r) and node[
            1] <= (obstacle[4] - r) and node[2] >= (obstacle[2] - r) and node[2] <= (obstacle[5] - r):
            return True
    return False


def rrt_star_bid_h_main(X_dimensions, x_init, x_goal, Obstacles=None, show=False):
    Q = np.array([(8, 4)])  # length of tree edges
    r = 2  # 检查与障碍物碰撞的最小边长度
    max_samples = 1024  # 最大采样节点数量
    rewire_count = 32  # optional, number of nearby branches to rewire
    prc = 0.1  # 每次生成新的节点后检查与目标的相连的概率
    if Obstacles is not None:
        if node_is_in_Obstacles(x_init, Obstacles, r):  # 与障碍物相交
            X = SearchSpace(X_dimensions, Obstacles)
            return X, None
        else:
            X = SearchSpace(X_dimensions, Obstacles)
            rrt = RRTStarBidirectionalHeuristic(X, Q, x_init, x_goal, max_samples, r, prc, rewire_count)
            path = rrt.rrt_star_bid_h()
    else:
        X = SearchSpace(X_dimensions, None)
        rrt = RRTStarBidirectionalHeuristic(X, Q, x_init, x_goal, max_samples, r, prc, rewire_count)
        path = rrt.rrt_star_bid_h()
    # plot
    if show == True:
        plot = Plot("rrt_star_bid_h_3d")
        # plot.plot_tree(X, rrt.trees)

        if path is not None:
            plot.plot_path(X, path)
        else:
            print("path is None")
        plot.plot_obstacles(X, Obstacles)
        plot.plot_start(X, x_init)
        plot.plot_goal(X, x_goal)
        plot.draw(auto_open=True)
    return X, path


if __name__ == '__main__':
    X_dimensions = np.array([(0, 100), (0, 100), (0, 100)])  # dimensions of Search Space
    # # obstacles
    # Obstacles = np.array(
    #     [(20, 20, 20, 40, 40, 40), (20, 20, 60, 40, 40, 80), (20, 60, 20, 40, 80, 40), (60, 60, 20, 80, 80, 40),
    #      (60, 20, 20, 80, 40, 40), (60, 20, 60, 80, 40, 80), (20, 60, 60, 40, 80, 80), (60, 60, 60, 80, 80, 80)])
    # x_init = (0, 0, 0)  # starting location
    # x_goal = (100, 100, 100)  # goal location

    Obstacles = np.array(
        [(10, 10, 30, 85, 50, 35), (30, 20, 0, 35, 25, 30), (60, 20, 0, 65, 25, 30),
         (30, 35, 0, 35, 40, 30), (60, 35, 0, 65, 40, 30)])
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
    rrt = RRTStarBidirectionalHeuristic(X, Q, x_init, x_goal, max_samples, r, prc, rewire_count)
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
