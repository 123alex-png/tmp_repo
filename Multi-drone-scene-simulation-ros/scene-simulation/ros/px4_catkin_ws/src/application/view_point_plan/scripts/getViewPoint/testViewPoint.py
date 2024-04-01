import numpy as np

from getViewPoint.getView import *
from RRT.rrt import RRT
from RRT.search_space import SearchSpace
from RRT.plotting import Plot
from RRT.geometry import *


if __name__ == '__main__':

    # 获取桥梁的视点
    # 桥梁构件
    # 桥面
    bridge_1 = [(10, 10, 30, 85, 50, 35)]
    # 四个桥墩
    bridge_2 = [(30, 20, 0, 35, 25, 30), (60, 20, 0, 65, 25, 30),
     (30, 35, 0, 35, 40, 30), (60, 35, 0, 65, 40, 30)]

    view_point_list = bridge_deck([10,10,35],[85,10,35],[85,50,35],[10,50,35],5,5)  # 桥面
    view_point_list += bridge_sidedeck([10,10,32],[85,10,32],-5,5)  # 桥侧面  问题

    view_point_list += bridge_squat([30,20,0],[35,20,0],[35,25,0],[30,25,0],
                                   [30,20,30],[35,20,30],[30,25,30],[30,25,30],5,5,8,8)   # 桥墩



    print(view_point_list)
    X_dimensions = np.array([(0, 100), (0, 100), (0, 100)])  # dimensions of Search Space
    Obstacles = np.array(
        [(10, 10, 30, 85, 50, 35), (30, 20, 0, 35, 25, 30), (60, 20, 0, 65, 25, 30),
         (30, 35, 0, 35, 40, 30), (60, 35, 0, 65, 40, 30)])

    # create Search Space
    X = SearchSpace(X_dimensions, Obstacles)
    # plot
    plot = Plot("rrt_3d")
    # plot.plot_tree(X, rrt.trees)
    plot.plot_obstacles(X, Obstacles)







    for point in view_point_list:
        plot.plot_goal(X,point)
    plot.draw(auto_open=True)
