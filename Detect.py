import matplotlib.pyplot as plt
from Env import Environment
import numpy as np

class Detect:
    def __init__(self, env="C:\\Users\\Mr.D\\Desktop\\A2\\env1.jpg"):
        """
        初始化右倾规划类。

        参数:
            env (str): 环境图像文件路径，默认值为指定路径。
        """
        self.env = Environment(env)  # 创建环境实例
        # self.env.start = (580, 1868)
        # self.env.goal = (961, 214)
        self.roads = []
        self.opposite_point = []
        self.roads_vector1 = []
        self.marking_points = []


    def show(self):
        plt.show()

    def show_opposite_point(self):
        self.opposite_point = np.load('opposite_point.npy', allow_pickle=True).item()  # 预处理的邻居点
        fig, ax = plt.subplots()
        # 绘制环境图像
        # 反转颜色：将值为 255 的地方变为 0（黑色），将值为 0 的地方变为 255（白色）
        ax.imshow(self.env.erode_env, cmap='gray')

        for i in self.opposite_point:
            point = [i, self.opposite_point[i]]
            point_x, point_y = zip(*point)
            ax.plot(point_y, point_x, 'b-', linewidth=1, label='opposite_point')

        # 为了避免重复的图例项，只添加一次路径标签
        handles, labels = ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax.legend(by_label.values(), by_label.keys())

        # 设置轴标签
        ax.set_xlabel('Y')
        ax.set_ylabel('X')


    def show_roads(self):
        self.roads = np.load('roads.npy', allow_pickle=True).item()  # 加载道路点集并转换为元组
        fig, ax = plt.subplots()
        # 绘制环境图像
        # 反转颜色：将值为 255 的地方变为 0（黑色），将值为 0 的地方变为 255（白色）
        ax.imshow(self.env.erode_env, cmap='gray')

        point_x, point_y = zip(*self.roads)
        ax.plot(point_y, point_x, 'b.', linewidth=1, label='roads')

        # 为了避免重复的图例项，只添加一次路径标签
        handles, labels = ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax.legend(by_label.values(), by_label.keys())

        # 设置轴标签
        ax.set_xlabel('Y')
        ax.set_ylabel('X')

    def show_roads_vector1(self):
        self.roads_vector1 = np.load('roads_vector1.npy', allow_pickle=True).item()  # 道路向量
        fig, ax = plt.subplots()
        # 绘制环境图像
        # 反转颜色：将值为 255 的地方变为 0（黑色），将值为 0 的地方变为 255（白色）
        ax.imshow(self.env.erode_env, cmap='gray')

        for i in self.roads_vector1:
            point = [i, (i[0] + self.roads_vector1[i][0], i[1] + self.roads_vector1[i][1])]
            point_x, point_y = zip(*point)
            ax.plot(point_y, point_x, 'b-', linewidth=1, label='roads_vector1')

        # 为了避免重复的图例项，只添加一次路径标签
        handles, labels = ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax.legend(by_label.values(), by_label.keys())

        # 设置轴标签
        ax.set_xlabel('Y')
        ax.set_ylabel('X')

    def show_marking_points(self):
        self.marking_points = np.load('marking_points.npy', allow_pickle=True).item()
        fig, ax = plt.subplots()
        # 绘制环境图像
        # 反转颜色：将值为 255 的地方变为 0（黑色），将值为 0 的地方变为 255（白色）
        ax.imshow(self.env.erode_env, cmap='gray')

        for i in self.marking_points:
            point = [i, self.opposite_point[i]]
            point_x, point_y = zip(*point)
            ax.plot(point_y, point_x, 'b-', linewidth=1, label='marking_points')

        # 为了避免重复的图例项，只添加一次路径标签
        handles, labels = ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax.legend(by_label.values(), by_label.keys())

        # 设置轴标签
        ax.set_xlabel('Y')
        ax.set_ylabel('X')


x =Detect()
x.show_opposite_point()
x.show_roads()
x.show_roads_vector1()
x.show_marking_points()
x.show()
# 保存数组到 txt 文件

