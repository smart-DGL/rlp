import numpy as np
import numpy as np
from math import comb

# 检测路径中方向变化的点，并返回这些控制点
def set_control_points(path, n):  # n用于增加曲线始末的控制点个数
    control_points = []
    for i in range(1, len(path) - 1):
        p0, p1, p2 = path[i - 1], path[i], path[i + 1]
        # 计算向量 v1 和 v2
        v1 = np.array([p1[0] - p0[0], p1[1] - p0[1]])
        v2 = np.array([p2[0] - p1[0], p2[1] - p1[1]])

        # 计算两个向量之间的夹角
        angle = np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))
        if angle > 0:  # 如果角度大于某个阈值（例如0.1弧度），则认为方向发生了变化
            control_points.append(path[i])
    if control_points:
        first = control_points[0]
        last = control_points[-1]

        if path.index(first) >= n:
            control_points[0:0] = path[path.index(first) - (n + 1):path.index(first)]
        else:
            control_points[0:0] = path[:path.index(first)]

        if path.index(last) <= len(path) - (n + 1):
            control_points.extend(path[path.index(last) + 1:path.index(last) + (n + 2)])
        else:
            control_points.extend(path[path.index(last):])
    return control_points

# 构建帕斯卡三角形的第 m 行，并返回该行中的第 n 个元素
def binomial_coefficient(m, n):
    return comb(m, n)

# 根据起始值、结束值和步长生成等差数列
def arithmetic_progression(a1, stop, step):
    progression = []
    while a1 <= stop:
        progression.append(a1)
        a1 += step
    return progression

# 计算 n 阶贝塞尔曲线在 t 处的第 i 个伯恩斯坦多项式的值
def bernstein_polynomial(n, i, t):
    return binomial_coefficient(n, i) * (t ** i) * ((1 - t) ** (n - i))






