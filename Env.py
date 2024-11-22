import random
import numpy as np
import cv2


class Environment:
    def __init__(self, env_png=r"C:\Users\Mr.D\Desktop\A\env3.jpg"):
        """
        初始化环境类。

        参数:
            env_png (str): 环境图像文件路径，默认值为指定路径。
        """

        self.read_env(env_png)  # 读取环境图像并进行预处理
        self.erode_env = self.erode(4)
        self.using_env = self.erode_env  # 算法中预规划使用膨胀后的地图
        self.rows = self.bin_env.shape[0]  # 获取地图行数
        self.columns = self.bin_env.shape[1]  # 获取地图列数
        self.start = None
        self.goal = None
        self.set_start_goal()  # 设置随机起点和终点

    def set_start_goal(self):
        """
        设定随机的起点和终点。

        该方法从非障碍物的位置中随机选择一个点作为起点，然后从剩余的非障碍物位置中随机选择一个点作为终点。
        """
        # 找到所有非障碍物的位置
        valid_positions = [(x, y) for x in range(self.rows) for y in range(self.columns) if self.bin_env[x][y] != 0]

        # 随机选择起点
        start_index = random.choice(valid_positions)
        self.start = start_index
        valid_positions.remove(start_index)

        # 从剩下的位置中选择终点
        goal_index = random.choice(valid_positions)
        self.goal = goal_index

    def rgb2gray(self, rgb_img):
        """
        将原始图像转为灰度图。

        参数:
            rgb_img (numpy.ndarray): 原始RGB图像。

        返回:
            numpy.ndarray: 转换后的灰度图。
        """
        gray = rgb_img[:, :, 0] * 0.295 + rgb_img[:, :, 1] * 0.587 + rgb_img[:, :, 2] * 0.114
        return gray.astype(np.uint8)

    def im_binary(self, gray_image, t=100):
        """
        将灰度图转为二值图（黑白图）。

        参数:
            gray_image (numpy.ndarray): 灰度图。
            t (int): 阈值，默认为80。

        返回:
            numpy.ndarray: 转换后的二值图。
        """
        binary_image = np.zeros(shape=(gray_image.shape[0], gray_image.shape[1]), dtype=np.uint8)
        for i in range(gray_image.shape[0]):
            for j in range(gray_image.shape[1]):
                if gray_image[i][j] > t:
                    binary_image[i][j] = 0
                else:
                    binary_image[i][j] = 1
        return binary_image

    def read_env(self, env_png):
        """
        读取环境图像并进行预处理。

        参数:
            env_png (str): 环境图像文件路径。
        """

        self.origin_env = np.array(cv2.imread(env_png))  # 读取环境图像
        self.gray_img = 255 - self.rgb2gray(self.origin_env)  # 将图像转换为灰度图，并反转颜色
        self.bin_env = self.im_binary(self.gray_img)  # 将灰度图转换为二值图

    def erode(self, kernel_size, iterations=2):
        """
        对二值图进行腐蚀操作。

        参数:
            kernel_size (int): 腐蚀核大小。
            iterations (int): 腐蚀迭代次数，默认为2。

        返回:
            numpy.ndarray: 腐蚀后的二值图。
        """
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        erode_env = cv2.erode(self.bin_env, kernel, iterations=iterations)
        return erode_env

    def dilate(self, kernel_size, iterations=2):
        """
        对二值图进行膨胀操作。

        参数:
            kernel_size (int): 膨胀核大小。
            iterations (int): 膨胀迭代次数，默认为2。

        返回:
            numpy.ndarray: 膨胀后的二值图。
        """
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        dilate_env = cv2.dilate(self.bin_env, kernel, iterations=iterations)
        return dilate_env

