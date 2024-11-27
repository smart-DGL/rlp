import copy
from scipy.spatial import distance
from Env import Environment
from Bezier import *



class RightLeaningPlanning:
    def __init__(self, env="C:\\Users\\Mr.D\\Desktop\\A2\\env1.jpg"):
        """
        初始化右倾规划类。

        参数:
            env (str): 环境图像文件路径，默认值为指定路径。
        """
        self.env = Environment(env)  # 创建环境实例
        # self.env.start = (634, 592)
        # self.env.goal = (359, 1685)
        print('start', self.env.start, 'goal', self.env.goal)
        self.start = self.get_vaild_point(self.env.start)
        self.goal = self.get_vaild_point(self.env.goal)
        self.roads = np.load('roads.npy', allow_pickle=True).item()
        self.opposite_point = np.load('opposite_point.npy', allow_pickle=True).item()  # 预处理的邻居点
        self.roads_vector1 = np.load('roads_vector1.npy', allow_pickle=True).item()  # 道路向量
        self.path = []  # 初始路径
        self.final_path = self.start_gaol()  # 最终路径
        if not self.final_path:
            self.right_normal_vectors = self.define_right_normal_vectors()  # 定义右侧法线向量
            self.nearest_start = self.find_nearest_point_roads(self.start)
            self.nearest_goal = self.find_nearest_point_roads(self.goal)  # 找到最近的起点和终点
            self.path = self.composite_a(self.nearest_start, self.nearest_goal)  # A* 搜索初始路径
            self.path.insert(0, self.env.start)
            self.path.append(self.env.goal)

    def get_vaild_point(self, point):
        if self.env.using_env[point] == 0:
            point = self.find_nearest_point_env(point)
        else:
            point = point
        return point

    def is_straight_line(self, point1, point2):
        # 计算两点之间的直线方程 y = mx + c
        x1, y1 = point1
        x2, y2 = point2

        if x2 - x1 == 0:  # 垂直线
            m = None
            c = x1
        elif y2 - y1 == 0:  # 水平线
            m = 0
            c = y1
        else:
            m = (y2 - y1) / (x2 - x1)
            c = y1 - m * x1

        def line_eq(x):
            if m is None:
                return c
            return m * x + c

        def check_neighbors(x, y):
            # 检查点 (x, y) 周围的四个点
            neighbors = [
                (x - 1, y), (x + 1, y),
                (x, y - 1), (x, y + 1)
            ]
            for nx, ny in neighbors:
                if 0 <= nx < self.env.bin_env.shape[0] and 0 <= ny < self.env.bin_env.shape[1]:
                    if self.env.bin_env[nx, ny] == 0:
                        return True
            return False

        def all_neighbors_zero(x, y):
            # 检查点 (x, y) 周围的四个点是否都为0
            neighbors = [
                (x - 1, y), (x + 1, y),
                (x, y - 1), (x, y + 1)
            ]
            for nx, ny in neighbors:
                if 0 <= nx < self.env.bin_env.shape[0] and 0 <= ny < self.env.bin_env.shape[1]:
                    if self.env.bin_env[nx, ny] != 0:
                        return False
            return True

        # 找到直线上最接近的整数点
        min_x, max_x = min(x1, x2), max(x1, x2)
        min_y, max_y = min(y1, y2), max(y1, y2)

        if m is None:  # 垂直线
            for y in range(min_y, max_y + 1):
                x = c
                if 0 <= x < self.env.bin_env.shape[0] and all_neighbors_zero(x, y):
                    return False
        elif m == 0:  # 水平线
            for x in range(min_x, max_x + 1):
                y = c
                if 0 <= y < self.env.bin_env.shape[1] and all_neighbors_zero(x, y):
                    return False
        else:  # 斜线
            for x in range(min_x, max_x + 1):
                y = int(round(line_eq(x)))
                if 0 <= y < self.env.bin_env.shape[1] and all_neighbors_zero(x, y):
                    return False

            for y in range(min_y, max_y + 1):
                x = int(round((y - c) / m))
                if 0 <= x < self.env.bin_env.shape[0] and all_neighbors_zero(x, y):
                    return False

        return True

    def start_gaol(self):
        if distance.euclidean(self.env.start, self.env.goal) <= 50:
            connect = self.is_straight_line(self.env.start, self.env.goal)
            if connect:
                return [self.env.start, self.env.goal]
            else:
                return []

    def define_right_normal_vectors(self):
        """
        定义八个方向上的右侧法线方向。
        """
        right_normal_vectors = {
            (1, 0): (0, -1),
            (1, 1): (1, -1),
            (0, 1): (1, 0),
            (-1, 1): (1, 1),
            (-1, 0): (0, 1),
            (-1, -1): (-1, 1),
            (0, -1): (-1, 0),
            (1, -1): (-1, -1)
        }
        return right_normal_vectors

    def get_point_in_ring(self, center, r):
        """ 获取以中心点为中心，扩展半径为 r 的所有邻居点 """
        cx, cy = center
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                if abs(dx) == r or abs(dy) == r:
                    yield cx + dx, cy + dy

    def find_nearest_point_roads(self, center):
        r = 1
        while True:
            valid_points = []
            for point in self.get_point_in_ring(center, r):
                if 0 <= point[0] < self.env.rows and 0 <= point[1] < self.env.columns and point in self.roads:
                    valid_points.append(point)

            if valid_points:
                if len(valid_points) == 1:
                    return valid_points[0]
                else:
                    # 返回与 center 直线距离最短的点
                    nearest_point = min(valid_points, key=lambda p: distance.euclidean(p, center))
                    return nearest_point
            r += 1

    def find_nearest_point_env(self, center):
        r = 1
        while True:
            valid_points = []
            for point in self.get_point_in_ring(center, r):
                if 0 <= point[0] < self.env.rows and 0 <= point[1] < self.env.columns and self.env.using_env[point] != 0:
                    valid_points.append(point)

            if valid_points:
                if len(valid_points) == 1:
                    return valid_points[0]
                else:
                    # 返回与 center 直线距离最短的点
                    nearest_point = min(valid_points, key=lambda p: distance.euclidean(p, center))
                    return nearest_point
            r += 1

    def manhattan_distance(self, p1, p2):
        """
        计算两点之间的曼哈顿距离。
        """
        return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

    def get_neighbors_in_roads(self, current, goal, g_score, f_score):
        """
        获取当前点在道路中的邻居点。

        参数:
            current (tuple): 当前点坐标。
            g_score (dict): 当前节点到起点的实际代价。
            f_score (dict): 当前节点到终点的估计总代价。

        返回:
            list: 邻居点列表。
        """
        neighbors = []
        directions = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


        for dx, dy in directions:
            neighbor = (current[0] + dx, current[1] + dy)
            if neighbor in self.roads:
                direction = (neighbor[0] - current[0], neighbor[1] - current[1])
                # 检查方向向量是否在字典中
                if direction in self.right_normal_vectors:
                    right_normal = self.right_normal_vectors[direction]
                    right_step = (neighbor[0] + right_normal[0], neighbor[1] + right_normal[1])
                    # 检查右侧一步是否是障碍物
                    if 0 <= right_step[0] < self.env.rows and 0 <= right_step[1] < self.env.columns:
                        if self.env.using_env[right_step] == 0:
                            tentative_g_score = g_score[current] + distance.euclidean(current, neighbor)
                            tentative_f_score = tentative_g_score + self.manhattan_distance(neighbor, goal)
                            if neighbor not in f_score or tentative_f_score < f_score[neighbor]:
                                neighbors.append(neighbor)

        neighbor = self.opposite_point.get(current)
        if neighbor and neighbor in self.roads:
            tentative_g_score = g_score[current] + distance.euclidean(current, neighbor)
            tentative_f_score = tentative_g_score + self.manhattan_distance(neighbor, goal)
            # 如果邻居点不在 f_score 中，或者新的 f_score 更小，则更新
            if neighbor not in f_score or tentative_f_score <= f_score[neighbor]:
                neighbors.append(neighbor)

        return neighbors

    def composite_a(self, start, goal):
        """
        使用 A* 算法在给定的地图中寻找从 start 到 goal 的最短路径。

        参数:
            start (tuple): 起点坐标。
            goal (tuple): 终点坐标。
            method (function): 获取邻居点的方法。
        返回:
            list: 从 start 到 goal 的最短路径，如果找不到路径则返回 None。
        """
        open_set = {start}
        closed_set = set()
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.manhattan_distance(start, goal)}  # 改为曼哈顿距离

        while open_set:
            # 选择 f_score 最小的点
            current = min(open_set, key=lambda x: f_score[x])

            if current == goal:
                # 构建并返回路径
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                return path[::-1]

            open_set.remove(current)
            closed_set.add(current)

            # 获取当前点的邻居点
            neighbors = self.get_neighbors_in_roads(current, goal, g_score, f_score)
            for neighbor in neighbors:
                if neighbor in closed_set:
                    continue

                tentative_g_score = g_score[current] + distance.euclidean(current, neighbor)
                if neighbor not in open_set or tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = g_score[neighbor] + self.manhattan_distance(neighbor, goal)  # 启发式函数改为曼哈顿距离
                    if neighbor not in open_set:
                        open_set.add(neighbor)

        return None  # 如果没有找到路径，返回 None