from RRT.geometry import *
from RRT.plotting import Plot
from RRT.rrt import RRT
from RRT.search_space import SearchSpace


def node_is_in_Obstacles(node, Obstacles, r):
    for i, obstacle in enumerate(Obstacles):
        if node[0] >= (obstacle[0] - r) and node[0] <= (obstacle[3] + r) \
                and node[1] >= (obstacle[1] - r) and node[1] <= (obstacle[4] - r) \
                and node[2] >= (obstacle[2] - r) and node[2] <= (obstacle[5] - r):
            return True
    return False


def rrt_main(X_dimensions, x_init, x_goal, Obstacles=None, show=False):

    Q = np.array([(8, 4)])  # length of tree edges
    r = 2  # 检查与障碍物碰撞的最小边长度
    max_samples = 1024  # 最大采样节点数量
    prc = 0.1  # 每次生成新的节点后检查与目标的相连的概率
    if Obstacles is not None:
        if node_is_in_Obstacles(x_init, Obstacles, r):  # 与障碍物相交
            X = SearchSpace(X_dimensions, Obstacles)
            return X, None
        else:
            X = SearchSpace(X_dimensions, Obstacles)
            rrt = RRT(X, Q, x_init, x_goal, max_samples, r, prc)
            path = rrt.rrt_search()
    else:
        X = SearchSpace(X_dimensions, None)
        rrt = RRT(X, Q, x_init, x_goal, max_samples, r, prc)
        path = rrt.rrt_search()
    # plot
    if show == True:
        plot = Plot("rrt_3d")
        # plot.plot_tree(X, rrt.trees)

        if path is not None:
            plot.plot_path(X, path)
        else:
            print("path is None")
        plot.plot_obstacles(X, Obstacles)
        plot.plot_start(X, x_init)
        plot.plot_goal(X, x_goal)
        print("path is draw")
        plot.draw(auto_open=True)
    return X, path


if __name__ == '__main__':

    X_dimensions = np.array([(0, 100), (0, 100),
                             (0, 100)])  # dimensions of Search Space
    Obstacles = np.array([(10, 10, 30, 85, 50, 35), (30, 20, 0, 35, 25, 30),
                          (60, 20, 0, 65, 25, 30), (30, 35, 0, 35, 40, 30),
                          (60, 35, 0, 65, 40, 30)])
    x_init = (20, 30, 0)  # starting location
    x_goal = (40, 30, 50)  # goal location
    # x_init = (30, 35, 0)  # starting location
    # x_goal = (10, 50, 30)  # goal location
    X, path = rrt_main(X_dimensions, Obstacles, x_init, x_goal, True)

    path_len = 0
    tmp_node = x_init
    if path is not None:
        for i in path:
            print((i[0], i[1], i[2]))
            path_len += dist_between_points(tmp_node, (i[0], i[1], i[2]))
            tmp_node = (i[0], i[1], i[2])

    print(path_len)
