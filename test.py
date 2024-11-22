import logging
from RLP2 import *
import matplotlib.pyplot as plt
import os

r = RightLeaningPlanning("C:\\Users\\Mr.D\\Desktop\\A\\env4.png")

if r.processed_path:
    fig, ax = plt.subplots()
    # 绘制环境图像
    # 反转颜色：将值为 255 的地方变为 0（黑色），将值为 0 的地方变为 255（白色）
    ax.imshow(r.env.bin_env, cmap='gray')

    # 绘制起点（绿色）
    start_x, start_y = r.env.start
    ax.plot(start_y, start_x, 'go', markersize=10, label='Start{}'.format((start_x, start_y)))

    # 绘制终点（红色）
    goal_x, goal_y = r.env.goal
    ax.plot(goal_y, goal_x, 'ro', markersize=10, label='Goal{}'.format((goal_x, goal_y)))

    path_x, path_y = zip(*r.processed_path)
    ax.plot(path_y, path_x, 'b.', linewidth=1, label='processed_path')

    # 为了避免重复的图例项，只添加一次路径标签
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys())

    # 设置轴标签
    ax.set_xlabel('Y')
    ax.set_ylabel('X')

else:
    print('no path')

if r.final_path:
    fig, ax = plt.subplots()
    # 绘制环境图像
    # 反转颜色：将值为 255 的地方变为 0（黑色），将值为 0 的地方变为 255（白色）
    ax.imshow(r.env.bin_env, cmap='gray')

    # 绘制起点（绿色）
    start_x, start_y = r.env.start
    ax.plot(start_y, start_x, 'go', markersize=10, label='Start{}'.format((start_x, start_y)))

    # 绘制终点（红色）
    goal_x, goal_y = r.env.goal
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
else:
    print('no final_path')

if r.path:
    fig, ax = plt.subplots()
    # 绘制环境图像
    # 反转颜色：将值为 255 的地方变为 0（黑色），将值为 0 的地方变为 255（白色）
    ax.imshow(r.env.bin_env, cmap='gray')

    # 绘制起点（绿色）
    start_x, start_y = r.env.start
    ax.plot(start_y, start_x, 'go', markersize=10, label='Start{}'.format((start_x, start_y)))

    # 绘制终点（红色）
    goal_x, goal_y = r.env.goal
    ax.plot(goal_y, goal_x, 'ro', markersize=10, label='Goal{}'.format((goal_x, goal_y)))

    path_x, path_y = zip(*r.path)
    ax.plot(path_y, path_x, 'b.', linewidth=1, label='path')

    # 为了避免重复的图例项，只添加一次路径标签
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys())

        # 设置轴标签
    ax.set_xlabel('Y')
    ax.set_ylabel('X')
else:
    print('no path')


plt.show()