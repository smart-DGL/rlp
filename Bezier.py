from Auxiliary_functions import *

class BezierN:
    def __init__(self, path, resolution=1000):
        self.path = path
        self.resolution = resolution  # 分辨率默认为10000
        control_points = set_control_points(self.path, 3)
        if not control_points:
            self.smoothing_path = self.path  # 说明原路径没有尖点，是直线，返回原path
        else:
            self.control_points = control_points
            self.n = len(self.control_points) - 1  # Bezier 曲线的阶数
            bezier_curve = self.get_points()
            self.smoothing_path = (path[:path.index(self.control_points[0])]
                                  + bezier_curve
                                  + path[path.index(self.control_points[-1]) + 1:])

    def get_point(self, t):
        # 初始化Bezier点
        bezier_point = [0, 0]
        for i in range(len(self.control_points)):
            # nth 阶Bezier曲线的参数方程
            bezier_point[0] += bernstein_polynomial(self.n, i, t) * (self.control_points[i][0])
            bezier_point[1] += bernstein_polynomial(self.n, i, t) * (self.control_points[i][1])
        return tuple(bezier_point)  # 返回计算得到的Bezier点，确保是元组形式

    def get_points(self):
        # 初始化Bezier曲线
        bezier_curve = []
        step = 1 / float(self.resolution)  # 根据分辨率计算步长

        # 对于从0到1的每个参数值，计算对应的Bezier点并添加到曲线中
        for parameter in arithmetic_progression(0, 1, step):
            bezier_curve.append(self.get_point(parameter))

        return bezier_curve  # 返回整个Bezier曲线，确保是元组形式