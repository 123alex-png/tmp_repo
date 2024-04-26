"""
@File: getView.py
@Author: lizeshan zhangruiheng
@Date: 2024-04-25
@Description: 从对应的巡检设施中提取视点
"""

import math
import numpy as np

"F:焦距, Sw:像幅宽, FOVw:地面宽"


def cal_h_by_Fov(F, Sw, FOVw):
    """

    :param F: 焦距
    :param Sw: 像幅宽
    :param FOVw: 摄影对象宽
    :return: 摄影高度
    """
    H = F * FOVw / Sw
    return H


"FOVl：地面长, O:前向重叠度"
"FOVl = 像幅长 * 地面宽 / 像幅宽"


def cal_len_by_FOV(Sl, FOVw, Sw, O):
    """

    :param Sl: 像幅长
    :param FOVw: 摄影对象宽
    :param Sw: 像幅宽
    :param O: 前向重叠度
    :return: 拍摄间隔点长度
    """
    FOVl = Sl * FOVw / Sw
    len = FOVl * (2 - O) - FOVl
    return len


"路面"
"""
d-----------c
|           |
|           |
|           |
a-----------b
近似水平
"""


def RoadSurface_cam(a, b, c, d, F, Sw, Sl, O):
    """

    :param a: 路面左下角
    :param b: 路面右下角
    :param c: 路面右上角
    :param d: 路面左上角
    ab, cd 所连的直线为公路
    :param F: 焦距
    :param Sw: 像幅宽
    :param Sl: 像幅长
    :param O: 前向重叠度
    :return: 提取的视点列表
    """
    FOVw = ((d[0] - a[0])**2 + (d[1] - a[1])**2)**0.5
    h = cal_h_by_Fov(F, Sw, FOVw)
    len = cal_len_by_FOV(Sl, FOVw, Sw, O)
    return RoadSurface(a, b, c, d, len, h)


def RoadSurface(a, b, c, d, len, h):
    """

    :param a: 路面左下角
    :param b: 路面右下角
    :param c: 路面右上角
    :param d: 路面左上角
    ab, cd 所连的线段为公路两侧
    :param len: 无人机摄影间隔长度
    :param h: 无人机摄影高度
    :return: 提取的视点列表
    """
    points_list = []
    length = pow((b[0] - a[0])**2 + (b[1] - a[1])**2, 0.5)
    curlength = 0
    while (curlength <= length):
        point = []
        point.append(a[0] + (b[0] - a[0]) * curlength / length)
        point.append(a[1] + (b[1] - a[1]) * curlength / length)
        point.append(a[2] + h)
        point1 = []
        point1.append(d[0] + (c[0] - d[0]) * curlength / length)
        point1.append(d[1] + (c[1] - d[1]) * curlength / length)
        point1.append(d[2] + h)
        points_list.append(point)
        '''points_list.append(point1)'''
        curlength = curlength + len
    if length % len != 0:
        point = []
        point.append(b[0])
        point.append(b[1])
        point.append(b[2] + h)
        point1 = []
        point1.append(c[0])
        point1.append(c[1])
        point1.append(c[2] + h)
        points_list.append(point)
        '''points_list.append(point1)'''
    return points_list


"隔离栅"


def Barrier(a, b, c, d, F, Sw, Sl, O):
    points_list = []
    return points_list


"防撞护栏"


def Guardrail():
    points_list = []
    return points_list


"桥面,包含桥面和桥侧面"
"同路面"


def Deck_cam(a, b, c, d, thickness, F, Sw, Sl, O):
    """

    :param a: 桥面左下角
    :param b: 桥面右下角
    :param c: 桥面右上角
    :param d: 桥面左上角
    :param thickness: 桥的厚度
    ab, cd所连的线段为桥的两侧
    :param F: 焦距
    :param Sw: 像幅宽
    :param Sl: 像幅长
    :param O: 前向重叠度
    :return: 提取的视点列表
    """
    FOVw = ((d[0] - a[0])**2 + (d[1] - a[1])**2)**0.5
    h = cal_h_by_Fov(F, Sw, FOVw)
    len = cal_len_by_FOV(Sl, FOVw, Sw, O)
    s1 = a
    s1[2] = s1[2] - thickness / 2
    e1 = b
    e1[2] = e1[2] - thickness / 2
    s2 = c
    s2[2] = s2[2] - thickness / 2
    e2 = d
    e2[2] = e2[2] - thickness / 2
    return Deck(a, b, c, d, len, h) + bridge_sidedeck_cam(
        s1, e1, Sw, F, thickness, Sl, O) + bridge_sidedeck_cam(
            s2, e2, Sw, F, thickness, Sl, O)


def Deck(a, b, c, d, len, h):
    """

    :param a: 桥面左下角
    :param b: 桥面右下角
    :param c: 桥面右上角
    :param d: 桥面左上角
    ab, cd所连的线段为桥的两侧
    :param len: 无人机摄影间隔长度
    :param h: 无人机摄影高度
    :return: 提取的视点列表
    """
    points_list = []
    length = pow((b[0] - a[0])**2 + (b[1] - a[1])**2, 0.5)
    curlength = 0
    while (curlength <= length):
        point = []
        point.append(a[0] + (b[0] - a[0]) * curlength / length)
        point.append(a[1] + (b[1] - a[1]) * curlength / length)
        point.append(a[2] + h)
        point1 = []
        point1.append(d[0] + (c[0] - d[0]) * curlength / length)
        point1.append(d[1] + (c[1] - d[1]) * curlength / length)
        point1.append(d[2] + h)
        points_list.append(point)
        points_list.append(point1)
        curlength = curlength + len
    if length % len != 0:
        point = []
        point.append(b[0])
        point.append(b[1])
        point.append(b[2] + h)
        point1 = []
        point1.append(c[0])
        point1.append(c[1])
        point1.append(c[2] + h)
        points_list.append(point)
        points_list.append(point1)
    return points_list


"桥侧面"
"近似竖直面,桥厚度忽略不计，视为线对象"
"然而视为线对像则与相机有关的参数计算不融洽，故此处考虑厚度，厚度即FOVw"


def bridge_sidedeck_cam(start, end, Sw, F, FOVw, Sl, O):
    """

    :param start: 桥侧面的起始点
    :param end: 桥侧面的终止点
    起始终止点都应取为中点
    :param Sw: 像幅宽
    :param F: 焦距
    :param FOVw: 桥的厚度
    :param Sl: 像幅长
    :param O: 前向重叠度
    :return: 提取的视点列表
    """
    h = cal_h_by_Fov(F, Sw, FOVw)
    len = cal_len_by_FOV(Sl, FOVw, Sw, O)
    return bridge_sidedeck(start, end, h, len)


def bridge_sidedeck(start, end, dis, len):
    """

    :param start: 桥侧面的起始点
    :param end: 桥侧面的终止点
    :param dis: 与桥侧面应保持的距离
    :param len: 无人机摄影间隔长度
    :return:
    """
    points_list = []
    bridge_length = pow(((end[0] - start[0])**2 + (end[1] - start[1])**2 +
                         (end[2] - start[2])**2), 0.5)
    bridge_length_2d = pow(((end[0] - start[0])**2 + (end[1] - start[1])**2),
                           0.5)
    dis_x = dis * (end[1] - start[1]) / bridge_length_2d
    dis_y = dis * (end[0] - start[0]) / bridge_length_2d
    curlength = 0
    while (curlength <= bridge_length):
        point = []
        point.append(start[0] +
                     (end[0] - start[0]) * curlength / bridge_length + dis_x)
        point.append(start[1] +
                     (end[1] - start[1]) * curlength / bridge_length + dis_y)
        point.append(start[2] +
                     (end[2] - start[2]) * curlength / bridge_length)
        points_list.append(point)
        curlength = curlength + len
    if bridge_length % len != 0:
        point = []
        point.append(end[0] + dis_x)
        point.append(end[1] + dis_y)
        point.append(end[2])
        points_list.append(point)
    return points_list


"桥伸缩缝"


def Joint(target, dis_z, min_dis_y, max_dis_y):
    """

        :param target: 目标点的坐标
        :param dis_z: 与目标点的竖向距离
        :param min_dis_y: 与目标点的最小纵向距离
        :param max_dis_y: 与目标点的最大横向距离
        :return: 提取的视点列表
        """
    points_list = []
    point = []
    point1 = []
    for dis_y in range(min_dis_y, max_dis_y):
        point.append(target[0])
        point.append(target[1] + dis_y)
        point.append(target[2] + dis_z)
        point1.append(target[0])
        point1.append(target[1] - dis_y)
        point1.append(target[2] + dis_z)
        if not hasobstacle(point) and not hasobstacle(point1):
            points_list.append(point)
            points_list.append(point1)
            break
    return points_list


"桥台"


def Abutment(target, dis_x, min_dis_y, max_dis_y):
    """

        :param target: 目标点的坐标
        :param dis_x: 与目标点的横向距离
        :param min_dis_y: 与目标点的最小纵向距离
        :param max_dis_y: 与目标点的最大横向距离
        :return: 提取的视点列表
        """
    points_list = []
    point = []
    point1 = []
    for dis_y in range(min_dis_y, max_dis_y):
        point.append(target[0] + dis_x)
        point.append(target[1] + dis_y)
        point.append(target[2])
        point1.append(target[0] + dis_x)
        point1.append(target[1] - dis_y)
        point1.append(target[2])
        if not hasobstacle(point) and not hasobstacle(point1):
            points_list.append(point)
            points_list.append(point1)
            break
    return points_list


"桥支座"


def Bearing(target, dis_x, min_dis_y, max_dis_y):
    """

    :param target: 目标点的坐标
    :param dis_x: 与目标点的横向距离
    :param min_dis_y: 与目标点的最小纵向距离
    :param max_dis_y: 与目标点的最大横向距离
    :return: 提取的视点列表
    """
    points_list = []
    point = []
    point1 = []
    for dis_y in range(min_dis_y, max_dis_y):
        point.append(target[0] + dis_x)
        point.append(target[1] + dis_y)
        point.append(target[2])
        point1.append(target[0] + dis_x)
        point1.append(target[1] - dis_y)
        point1.append(target[2])
        if not hasobstacle(point) and not hasobstacle(point1):
            points_list.append(point)
            points_list.append(point1)
            break
    return points_list


"桥墩"
"由四个近似竖直面构成"
"底面abcd,表面a1b1c1d1,八个点确定桥墩这一长方体"
"坐标顺序需确定以确定远离的方向"


def Pier_cam(a, b, c, d, a1, b1, c1, d1, F, Sw, Sl, O):
    """

    :param a: 桥墩底面左下角
    :param b: 桥墩底面右下角
    :param c: 桥墩底面右上角
    :param d: 桥墩底面左上角
    :param a1: 桥墩表面左下角
    :param b1: 桥表底面右下角
    :param c1: 表墩底面右上角
    :param d1: 桥墩表面左上角
    :param F: 焦距
    :param Sw: 像幅宽
    :param Sl: 像幅长
    :param O: 前向重叠度
    :return: 提取的视点列表
    """
    FOVw1 = ((b[0] - a[0])**2 + (b[1] - a[1])**2)**0.5
    FOVw2 = ((d[0] - a[0])**2 + (d[1] - a[1])**2)**0.5
    h1 = cal_h_by_Fov(F, Sw, FOVw1)
    h2 = cal_h_by_Fov(F, Sw, FOVw2)
    len1 = cal_len_by_FOV(Sl, FOVw1, Sw, O)
    len2 = cal_len_by_FOV(Sl, FOVw2, Sw, O)
    return Pier(a, b, c, d, a1, b1, c1, d1, h1, h2, len1, len2)


def Pier(a, b, c, d, a1, b1, c1, d1, dis1, dis2, len1, len2):
    """

    :param a: 桥墩底面左下角
    :param b: 桥墩底面右下角
    :param c: 桥墩底面右上角
    :param d: 桥墩底面左上角
    :param a1: 桥墩表面左下角
    :param b1: 桥表底面右下角
    :param c1: 表墩底面右上角
    :param d1: 桥墩表面左上角
    :param dis1: 与abb1a1和cdd1c1面应保持的距离
    :param dis2: 与bcc1b1和daa1d1面应保持的距离
    :param len1: 横向间隔距离
    :param len2: 竖向间隔距离
    :return: 提取的视点列表
    """
    points_list = []

    points_list = points_list + Pier_sidedeck(a,b,b1,a1,dis1,len1,len2,c,d,d1,c1) + \
                  Pier_sidedeck(b,c,c1,b1,dis2,len1,len2,d,a,a1,d1) + \
                  Pier_sidedeck(c,d,d1,c1,dis1,len1,len2,a,b,b1,a1) + \
                  Pier_sidedeck(d,a,a1,d1,dis2,len1,len2,b,c,c1,b1)

    return points_list


"对桥墩的单一面进行视点提取"


def Pier_sidedeck(a, b, c, d, dis, len1, len2, a1, b1, c1, d1):
    """

    :param a: 桥墩侧面左下角
    :param b: 桥墩侧面右下角
    :param c: 桥墩侧面右上角
    :param d: 桥墩侧面左上角
    :param dis: 无人机应与该面保持的距离
    :param len: 无人机摄影间隔长度
    :return: 提取的视点列表
    """
    # print(f"当前提取的桥墩面为:{a},{b},{c},{d}")
    points_list = []
    points = []
    points.append(a)
    points.append(b)
    points.append(c)
    normal = compute_plane_normal(points)
    print(normal)
    width = pow(((b[0] - a[0])**2 + (b[1] - a[1])**2), 0.5)
    length = math.fabs(c[2] - b[2])
    dis_x = dis * normal[0]
    dis_y = dis * normal[1]
    point = []
    point.append(a[0] + dis_x)
    point.append(a[1] + dis_y)
    point.append(a[2])
    if (point2area_distance(point, a1, b1, c1)) < dis:
        dis_x = -dis_x
        dis_y = -dis_y
    curlength = len2 * 3 / 2
    "靠近桥下危险，不对最低点提取视点"
    while (curlength < length):
        curwidth = len1 / 2
        while (curwidth < width):
            point = []
            point.append(b[0] - (b[0] - a[0]) * curwidth / width + dis_x)
            point.append(b[1] - (b[1] - a[1]) * curwidth / width + dis_y)
            point.append(c[2] - (c[2] - b[2]) * curlength / length)
            points_list.append(point)
            curwidth = curwidth + len1
        curlength = curlength + len2
    return points_list


"边坡面"
"""
d-----------c
|           |
|           |
|           |
a-----------b
非水平面,非竖直面
"""


def SlopeSurface_cam(a, b, c, d, F, length, width, Sw):
    """

    :param a: 边坡底左下角
    :param b: 边坡底右下角
    :param c: 边坡顶右上角
    :param d: 边坡顶左上角
    :param F: 焦距
    :param length: 边坡栅格的长度
    :param width: 边坡栅格的宽度
    :param Sw: 像幅宽
    :return: 提取的视点列表
    """
    FOVw = length
    h = cal_h_by_Fov(F, Sw, FOVw)
    return SlopeSurface(a, b, c, d, length, width, h)


def SlopeSurface(a, b, c, d, length, width, dis):
    """

    :param a: 边坡底左下角
    :param b: 边坡底右下角
    :param c: 边坡顶右上角
    :param d: 边坡顶左上角
    :param length: 边坡栅格的长度
    :param width: 边坡栅格的宽度
    :param dis: 无人机摄影时应与边坡保持的垂直距离
    :return: 提取的视点列表
    """
    points = []
    points.append(a)
    points.append(b)
    points.append(c)
    normal = compute_plane_normal(points)
    if normal[2] < 0:
        normal = normal * (-1)
    points_list = []
    search_list = []
    slope_length = pow(
        ((b[0] - a[0])**2 + (b[1] - a[1])**2 + (b[2] - a[2])**2), 0.5)
    slope_width = pow(((c[0] - b[0])**2 + (c[1] - b[1])**2 + (c[2] - b[2])**2),
                      0.5)
    num_length = slope_length / length
    if slope_length % length != 0:
        num_length = num_length + 1
    num_width = slope_width / width
    if slope_width % width != 0:
        num_width = num_width + 1
    for i in range(int(num_length)):
        for j in range(int(num_width)):
            cur_x = a[0] + (i + 0.5) / num_length * (b[0] - a[0]) + (
                j + 0.5) / num_width * (d[0] - a[0])
            cur_y = a[1] + (i + 0.5) / num_length * (b[1] - a[1]) + (
                j + 0.5) / num_width * (d[1] - a[1])
            cur_z = a[2] + (i + 0.5) / num_length * (b[2] - a[2]) + (
                j + 0.5) / num_width * (d[2] - a[2])
            point = []
            point.append(cur_x)
            point.append(cur_y)
            point.append(cur_z)
            search_list.append(point)
    '''
    cos = pow(((c[0] - b[0]) ** 2 + (c[1] - b[1]) ** 2),0.5)/slope_width
    sin = pow(1 - cos * cos,0.5)
    '''
    dis_x = dis * normal[0]
    dis_y = dis * normal[1]
    dis_z = dis * normal[2]
    for point_sc in search_list:
        hx = point_sc[0] + dis_x
        hy = point_sc[1] + dis_y
        hz = point_sc[2] + dis_z
        point = []
        point.append(hx)
        point.append(hy)
        point.append(hz)
        points_list.append(point)
    return points_list


"边坡底"


def SlopeBottom_cam(a, b, c, d, F, Sw, Sl, O):
    """

    :param a: 边坡底面左下角
    :param b: 边坡底面右下角
    :param c: 边坡底面右上角
    :param d: 边坡底面左上角
    ab, cd 所连的直线为公路
    :param F: 焦距
    :param Sw: 像幅宽
    :param Sl: 像幅长
    :param O: 前向重叠度
    :return: 提取的视点列表
    """
    FOVw = ((d[0] - a[0])**2 + (d[1] - a[1])**2)**0.5
    h = cal_h_by_Fov(F, Sw, FOVw)
    len = cal_len_by_FOV(Sl, FOVw, Sw, O)
    return RoadSurface(a, b, c, d, len, h)


def SlopeBottom(a, b, c, d, len, h):
    """
    
    :param a: 路面左下角
    :param b: 路面右下角
    :param c: 路面右上角
    :param d: 路面左上角
    ab, cd 所连的线段为公路两侧
    :param len: 无人机摄影间隔长度
    :param h: 无人机摄影高度
    :return: 提取的视点列表
    """
    points_list = []
    length = pow((b[0] - a[0])**2 + (b[1] - a[1])**2, 0.5)
    curlength = 0
    while (curlength <= length):
        point = []
        point.append(a[0] + (b[0] - a[0]) * curlength / length)
        point.append(a[1] + (b[1] - a[1]) * curlength / length)
        point.append(a[2] + h)
        point1 = []
        point1.append(d[0] + (c[0] - d[0]) * curlength / length)
        point1.append(d[1] + (c[1] - d[1]) * curlength / length)
        point1.append(d[2] + h)
        points_list.append(point)
        points_list.append(point1)
        curlength = curlength + len
    if length % len != 0:
        point = []
        point.append(b[0])
        point.append(b[1])
        point.append(b[2] + h)
        point1 = []
        point1.append(c[0])
        point1.append(c[1])
        point1.append(c[2] + h)
        points_list.append(point)
        points_list.append(point1)
    return points_list


"截水沟"


def SlopeDitch():
    points_list = []
    return points_list


"是否存在障碍"


def hasobstacle(point):
    return False


def compute_plane_normal(points):
    # 选取三个非共线的点
    p1, p2, p3 = points[:3]

    # 计算两个向量
    v1 = [p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]]
    v2 = [p3[0] - p1[0], p3[1] - p1[1], p3[2] - p1[2]]
    # # 求反向
    # v1 = p1 - p2
    # v2 = p1 -p3

    # 计算法向量
    normal = np.cross(v1, v2)

    normal = normal.astype(np.float32)
    # 标准化法向量
    normal /= np.linalg.norm(normal)

    return normal


def define_area(point1, point2, point3):
    """
    法向量    ：n={A,B,C}
    空间上某点：p={x0,y0,z0}
    点法式方程：A(x-x0)+B(y-y0)+C(z-z0)=Ax+By+Cz-(Ax0+By0+Cz0)
    https://wenku.baidu.com/view/12b44129af45b307e87197e1.html
    :param point1:
    :param point2:
    :param point3:
    :param point4:
    :return:（Ax, By, Cz, D）代表：Ax + By + Cz + D = 0
    """
    point1 = np.asarray(point1)
    point2 = np.asarray(point2)
    point3 = np.asarray(point3)
    AB = np.asmatrix(point2 - point1)
    AC = np.asmatrix(point3 - point1)
    N = np.cross(AB, AC)  # 向量叉乘，求法向量
    # Ax+By+Cz
    Ax = N[0, 0]
    By = N[0, 1]
    Cz = N[0, 2]
    D = -(Ax * point1[0] + By * point1[1] + Cz * point1[2])
    return Ax, By, Cz, D


def point2area_distance(point1, point2, point3, point4):
    """
    :param point1:数据框的行切片，三维
    :param point2:
    :param point3:
    :param point4:
    :return:点到面的距离
    """
    Ax, By, Cz, D = define_area(point1, point2, point3)
    mod_d = Ax * point4[0] + By * point4[1] + Cz * point4[2] + D
    mod_area = np.sqrt(np.sum(np.square([Ax, By, Cz])))
    d = abs(mod_d) / mod_area
    return d
