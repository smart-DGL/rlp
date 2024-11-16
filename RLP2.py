import copy
from scipy.spatial import distance
from Env import Environment
from Bezier import *



class RightLeaningPlanning:
    def __init__(self, env="C:\\Users\\Mr.D\\Desktop\\A\\env2.jpg"):
        """
        初始化右倾规划类。

        参数:
            env (str): 环境图像文件路径，默认值为指定路径。
        """
        self.env = Environment(env)  # 创建环境实例
        # self.env.start = (890, 86)
        # self.env.goal = (508, 1766)
        print('start', self.env.start, 'goal', self.env.goal)
        self.start = self.get_vaild_point(self.env.start)
        self.goal = self.get_vaild_point(self.env.goal)
        self.roads = np.load('roads.npy', allow_pickle=True).item()
        self.opposite_point = np.load('opposite_point.npy', allow_pickle=True).item()  # 预处理的邻居点
        self.roads_vector1 = np.load('roads_vector1.npy', allow_pickle=True).item()  # 道路向量
        self.marking_points = np.load('marking_points.npy', allow_pickle=True).item()
        self.path = []  # 初始路径
        self.processed_path = []  # 处理过的路径
        self.reprocessed_path = []
        self.final_path = self.start_gaol()  # 最终路径
        if not self.final_path:
            self.right_normal_vectors = self.define_right_normal_vectors()  # 定义右侧法线向量
            # print('right_normal_vectors is ok')
            self.nearest_start = self.find_nearest_point_roads(self.start)
            # print('nearest_start is ok:', self.nearest_start)
            self.nearest_goal = self.find_nearest_point_roads(self.goal)  # 找到最近的起点和终点
            # print('nearest_goal is ok', self.nearest_goal)
            self.path = self.composite_a(self.nearest_start, self.nearest_goal,
                                         self.get_neighbors_in_roads)  # A* 搜索初始路径
            # print('path is ok', self.path)
            self.processed_path = self.process_path(self.path)  # 处理路径
            # print('process_path is ok', self.processed_path)
            if self.processed_path:
                # self.reprocessed_path = self.reprocess_path(self.processed_path)
                self.final_path = self.connect_path(self.processed_path)  # 连接路径
            else:
                self.env.using_env = self.env.bin_env
                self.final_path = self.composite_a(self.env.start, self.env.goal, self.get_neighbors)
            # print('connect_path is ok', self.final_path)

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
        if distance.euclidean(self.env.start, self.env.goal) <= 100:
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
            ((-1, 0), (0, 1)): (1, 1),  # 上 -> 右 -> 右下
            ((-1, 0), (0, -1)): (-1, 1),  # 上 -> 左 -> 右上
            ((-1, 0), (-1, 0)): (0, 1),  # 上 -> 上 -> 右
            ((0, -1), (1, 0)): (-1, -1),  # 左 -> 下 -> 左上
            ((0, -1), (-1, 0)): (-1, 1),  # 左 -> 上 -> 右上
            ((0, -1), (0, -1)): (-1, 0),  # 左 -> 左 -> 上
            ((1, 0), (0, -1)): (-1, -1),  # 下 -> 左 -> 左上
            ((1, 0), (0, 1)): (1, -1),  # 下 -> 右 -> 左下
            ((1, 0), (1, 0)): (0, -1),  # 下 -> 下 -> 左
            ((0, 1), (-1, 0)): (1, 1),  # 右 -> 上 -> 右下
            ((0, 1), (1, 0)): (1, -1),  # 右 -> 下 -> 左下
            ((0, 1), (0, 1)): (1, 0),  # 右 -> 右 -> 下
            ((-1, 1), (1, 1)): (1, 0),  # 右上 -> 右下 -> 下
            ((-1, 1), (-1, -1)): (0, 1),  # 右上 -> 左上 -> 右
            ((-1, 1), (-1, 1)): (1, 1),  # 右上 -> 右上 -> 右下
            ((-1, -1), (1, -1)): (-1, 0),  # 左上 -> 左下 -> 左
            ((-1, -1), (-1, 1)): (0, 1),  # 左上 -> 右上 -> 右
            ((-1, -1), (-1, 1)): (-1, 1),  # 左上 -> 左上 -> 右上
            ((1, -1), (1, 1)): (0, -1),  # 左下 -> 右下 -> 左
            ((1, -1), (-1, -1)): (-1, 0),  # 左下 -> 左上 -> 上
            ((1, -1), (1, -1)): (-1, -1),  # 左下 -> 左下 -> 左上
            ((1, 1), (1, -1)): (0, -1),  # 右下 -> 左下 -> 左
            ((1, 1), (-1, 1)): (1, 0),  # 右下 -> 右上 -> 下
            ((1, 1), (1, 1)): (1, -1),  # 右下 -> 右下 -> 左下
        }
        return right_normal_vectors

    def get_point_in_ring(self, center, r):
        """ 获取以中心点为中心，扩展半径为 r 的所有邻居点 """
        cx, cy = center
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                if abs(dx) == r or abs(dy) == r:
                    yield cx + dx, cy + dy

    def get_point_in_region(self, center, r):
        """ 获取以中心点为中心，扩展半径为 r的方形内的去中心点的所有邻居点 """
        cx, cy = center
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                if dx == dy == 0:
                    continue
                else:
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
        # directions = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
        directions = [(-1, 0), (0, -1), (0, 1), (1, 0)]


        for dx, dy in directions:
            neighbor = (current[0] + dx, current[1] + dy)
            if neighbor in self.roads:
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

    def get_neighbors(self, current, goal, g_score, f_score):
        """
        获取当前点的所有邻居点（考虑环境边界）。

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
            if 0 <= neighbor[0] < self.env.rows and 0 <= neighbor[1] < self.env.columns:
                if self.env.using_env[neighbor] != 0:
                    tentative_g_score = g_score[current] + distance.euclidean(current, neighbor)
                    tentative_f_score = tentative_g_score + self.manhattan_distance(neighbor, goal)
                    if neighbor not in f_score or tentative_f_score <= f_score[neighbor]:
                        neighbors.append(neighbor)

        return neighbors

    def composite_a(self, start, goal, method):
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
            neighbors = method(current, goal, g_score, f_score)
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

    def process_node(self, predecessor, current, successor, processed_path):
        """
        处理节点，根据右侧法线方向进行调整。

        参数:
            predecessor (tuple): 前一个节点坐标。
            current (tuple): 当前节点坐标。
            successor (tuple): 后继节点坐标。
        返回:
            tuple: 调整后的新点坐标，如果没有合适的点则返回 None。
        """
        direction1 = (current[0] - predecessor[0], current[1] - predecessor[1])
        direction2 = (successor[0] - current[0], successor[1] - current[1])

        # 检查方向向量是否在字典中
        if (direction1, direction2) in self.right_normal_vectors:
            right_normal = self.right_normal_vectors[(direction1, direction2)]
            right_step = (current[0] + right_normal[0], current[1] + right_normal[1])

            # 检查右侧一步是否是障碍物
            if 0 <= right_step[0] < self.env.rows and 0 <= right_step[1] < self.env.columns:
                if self.env.using_env[right_step] == 0:
                    if current in self.roads_vector1:
                        if current not in processed_path:
                            return current
                else:
                    if current in self.opposite_point:
                        if current not in self.marking_points:
                            if self.opposite_point[current] not in processed_path:
                                return self.opposite_point[current]
        return None

    def process_path(self, path):
        """
        处理路径，根据右侧法线方向进行调整。
        """
        processed_path = []
        if path and len(path) > 2:
            for i in range(1, len(path) - 1):
                predecessor = path[i - 1]
                current = path[i]
                successor = path[i + 1]
                # 对当前节点进行处理
                new_point = self.process_node(predecessor, current, successor, processed_path)
                if new_point:
                    # 如果新点有效，则添加到处理过的路径中
                    processed_path.append(new_point)

        return processed_path

    def recreate_cluster(self, cluster_start, cluster_end):
        if self.manhattan_distance(cluster_start, cluster_end) >= 10:
            path = self.composite_a(cluster_start, cluster_end, self.get_neighbors_in_roads)
            if path:
                path.remove(cluster_start)
                path.remove(cluster_end)
                processed_path = self.process_path(path)
                if processed_path:
                    return processed_path
        else:
            return None

    def reprocess_path(self, processed_path):
        past = copy.deepcopy(processed_path)
        past_ = set(past)
        dict_ = {}
        for i in range(len(past) - 1):

            current_node = past[i]
            next_node = past[i + 1]

            path = self.recreate_cluster(current_node, next_node)
            # print('cluster_start', cluster_start, 'cluster_end', cluster_end)
            # print('path', path)
            if path:
                path_ = set(path)
                if not bool(path_ & past_):
                    dict_[i + 1] = path
        if dict_:
            for index in reversed(list(dict_.keys())):
                past[index: index] = dict_[index]

        if past == processed_path:
            return past
        else:
            return self.reprocess_path(past)

    def re_a_star(self, cluster_start, cluster_end):
        path = self.composite_a(cluster_start, cluster_end, self.get_neighbors)
        x = True
        if path:
            path.remove(cluster_start)
            path.remove(cluster_end)
            for point in path:
                if x:
                    for node in self.get_point_in_ring(point, 1):
                        if self.env.using_env[node] == 0:
                            self.env.using_env = self.env.erode(4)
                            cluster_start = self.get_vaild_point(cluster_start)  # 当前簇的最后一个点
                            cluster_end = self.get_vaild_point(cluster_end)  # 下一个簇的第一个点
                            path = self.composite_a(cluster_start, cluster_end, self.get_neighbors)
                            path.remove(cluster_start)
                            path.remove(cluster_end)
                            self.env.using_env = self.env.bin_env
                            x = False
                            break
                else:
                    break
        return path

    def connect_path(self, reprocessed_path):
        past = copy.deepcopy(reprocessed_path)
        past.insert(0, self.env.start)
        past.append(self.env.goal)
        dict_ = {}
        self.env.using_env = self.env.bin_env
        # 遍历 cutting_path 中的每个簇
        for index in range(len(past) - 1):

            current_node = past[index]
            next_node = past[index + 1]

            connect = self.is_straight_line(current_node, next_node)
            if connect:
                if distance.euclidean(current_node, next_node) >= 50:
                    path = self.re_a_star(current_node, next_node)
                    if path:
                        dict_[index + 1] = path
            else:
                path = self.re_a_star(current_node, next_node)
                if path:
                    dict_[index + 1] = path
        # 添加最后一个簇的所有点到最终路径中
        for i in reversed(list(dict_.keys())):
            past[i: i] = dict_[i]
        final_path = list(dict.fromkeys(past))
        return final_path