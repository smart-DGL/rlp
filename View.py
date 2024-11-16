import logging
from RLP2 import *
from Env import Environment
import matplotlib.pyplot as plt
import os

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(message)s')

for j in range(1000):
    r = RightLeaningPlanning()
    if r.final_path:
        fig, ax = plt.subplots()
        # 绘制环境图像
        # 反转颜色：将值为 255 的地方变为 0（黑色），将值为 0 的地方变为 255（白色）
        ax.imshow(r.env.bin_env, cmap='gray')

        # 绘制起点（绿色）
        start_x, start_y = r.start
        ax.plot(start_y, start_x, 'go', markersize=10, label='Start{}'.format((start_x, start_y)))

        # 绘制终点（红色）
        goal_x, goal_y = r.goal
        ax.plot(goal_y, goal_x, 'ro', markersize=10, label='Goal{}'.format((goal_x, goal_y)))

        path_x, path_y = zip(*r.final_path)
        ax.plot(path_y, path_x, 'b-', linewidth=1, label='final_path')

        # 为了避免重复的图例项，只添加一次路径标签
        handles, labels = ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        ax.legend(by_label.values(), by_label.keys())

        # 设置轴标签
        ax.set_xlabel('Y')
        ax.set_ylabel('X')

    plt.savefig('C:\\Users\\Mr.D\\Desktop\\A\\test\\temp{}.svg'.format(j), dpi=1800, format='svg')
    plt.close()
    print(j)

