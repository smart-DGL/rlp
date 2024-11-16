from Env import *
from scipy.spatial import distance

class Roads:
    def __init__(self, env=r"C:\Users\Mr.D\Desktop\A\env2.jpg"):
        self.env = Environment(env)
        self.roads = set()  # 初始化 roads
        self.roads_vector1 = {}  # 初始化 vector1 字典
        self.opposite_point = {}
        self.marking_points = set()
        self.num = 20
        # self.roads = np.load('roads.npy', allow_pickle=True)  # 加载道路点集并转换为元组
        # self.preprocessed_neighbors = np.load('preprocessed_neighbors.npy', allow_pickle=True).item()  # 预处理的邻居点
        # self.roads_vector1 = np.load('roads_vector1.npy', allow_pickle=True).item()  # 道路向量
        # self.roads_dis = np.load('roads_dis.npy', allow_pickle=True).item()
        self.find_roads()
        self.calculate_vector1()
        self.get_opposite_point()
        self.get_marking_points_in_area((746, 671), (811, 740))
        self.get_marking_points_in_area((671, 727), (760, 808))
        self.single((544, 740))
        self.single((446, 1260))
        self.single((444, 1264))
        self.save('roads.npy', self.roads)  # 包含了拐角点的roads
        self.save('roads_vector1.npy', self.roads_vector1)
        self.save('opposite_point.npy', self.opposite_point)
        self.save('marking_points.npy', self.marking_points)



    def find_roads(self):
        directions = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
        for x in range(self.env.rows):
            for y in range(self.env.columns):
                if self.env.using_env[x, y] == 0:  # 检查当前点是否为障碍物
                    for dx, dy in directions:
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < self.env.rows and 0 <= ny < self.env.columns and self.env.using_env[nx, ny] != 0:
                            self.roads.add((x, y))  # 使用 add 方法将点添加到集合中
                            break

    def is_in_straight_line(self, direction1, direction2):
        """
        判断两个方向是否在一条直线上。

        参数:
            direction1 (tuple): 第一个方向向量。
            direction2 (tuple): 第二个方向向量。

        返回:
            bool: 如果两个方向在同一直线上则返回True，否则返回False。
        """
        return direction1 == (-direction2[0], -direction2[1])

    def get_normal_direction(self, point, neighbor):
        """
        计算从一个点到邻居点的方向的法线方向。

        参数:
            point (tuple): 当前点坐标。
            neighbor (tuple): 邻居点坐标。

        返回:
            tuple: 法线方向。
        """
        dx, dy = neighbor[0] - point[0], neighbor[1] - point[1]
        return (dy, -dx)  # 法线方向

    def calculate_vector1(self):
        """
        遍历 self.roads 中的所有点，记录符合条件的法线方向。
        """
        directions = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]
        right_angle_directions = {
            ((-1, 0), (0, 1)): (-1, 1),  # 上方 - 右方 -> 右上方
            ((-1, 0), (0, -1)): (-1, -1),  # 上方 - 左方 -> 左上方
            ((1, 0), (0, 1)): (1, 1),  # 下方 - 右方 -> 右下方
            ((1, 0), (0, -1)): (1, -1),  # 下方 - 左方 -> 左下方
            ((0, 1), (1, 0)): (1, 1),  # 右方 - 下方 -> 右下方
            ((0, 1), (-1, 0)): (-1, 1),  # 右方 - 上方 -> 右上方
            ((0, -1), (1, 0)): (1, -1),  # 左方 - 下方 -> 左下方
            ((0, -1), (-1, 0)): (-1, -1),  # 左方 - 上方 -> 左上方
            ((-1, 1), (1, 1)): (0, 1),  # 左上方 - 右上方 -> 右方
            ((-1, -1), (1, -1)): (0, -1),  # 左下方 - 右下方 -> 左方
            ((-1, 1), (-1, -1)): (-1, 0),  # 左上方 - 左下方 -> 上方
            ((1, 1), (1, -1)): (1, 0)  # 右上方 - 右下方 -> 下方
        }

        for point in self.roads:
            x, y = point
            neighbors = []
            for dx, dy in directions:
                nx, ny = x + dx, y + dy
                if 0 <= nx < self.env.rows and 0 <= ny < self.env.columns and (nx, ny) in self.roads:
                    neighbors.append((nx, ny))

            if len(neighbors) >= 2:
                # 检查邻居点的方向是否在一条直线上
                directions_set = set()
                for neighbor in neighbors:
                    direction = (neighbor[0] - x, neighbor[1] - y)
                    directions_set.add(direction)

                if all(self.is_in_straight_line(d1, d2) for d1 in directions_set for d2 in directions_set if d1 != d2):
                    # 计算法线方向
                    normal_direction = self.get_normal_direction(point, neighbors[0])
                    # 确保法线方向指向值为0的一边
                    nx, ny = x + normal_direction[0], y + normal_direction[1]
                    if 0 <= nx < self.env.rows and 0 <= ny < self.env.columns and self.env.using_env[nx, ny] != 0:
                        self.roads_vector1[point] = normal_direction
                    else:
                        self.roads_vector1[point] = (-normal_direction[0], -normal_direction[1])
                else:
                    # 检查是否形成直角
                    for i in range(len(neighbors)):
                        for j in range(i + 1, len(neighbors)):
                            d1 = (neighbors[i][0] - x, neighbors[i][1] - y)
                            d2 = (neighbors[j][0] - x, neighbors[j][1] - y)
                            if (d1, d2) in right_angle_directions:
                                normal_direction = right_angle_directions[(d1, d2)]
                                nx, ny = x + normal_direction[0], y + normal_direction[1]
                                if 0 <= nx < self.env.rows and 0 <= ny < self.env.columns and self.env.using_env[nx, ny] != 0:
                                    self.roads_vector1[point] = normal_direction
                                break
                        else:
                            continue
                        break

            elif len(neighbors) == 1:
                # 只有一个邻居点
                normal_direction = self.get_normal_direction(point, neighbors[0])
                # 确保法线方向指向值为0的一边
                nx, ny = x + normal_direction[0], y + normal_direction[1]
                if 0 <= nx < self.env.rows and 0 <= ny < self.env.columns and self.env.using_env[nx, ny] != 0:
                    self.roads_vector1[point] = normal_direction
                else:
                    self.roads_vector1[point] = (-normal_direction[0], -normal_direction[1])

    def get_opposite_point(self):
        for point in self.roads_vector1.keys():
            for i in range(1, (self.env.rows + self.env.columns) // self.num):
                next_point = (point[0] + self.roads_vector1[point][0] * i, point[1] + self.roads_vector1[point][1] * i)
                if 0 <= next_point[0] < self.env.rows and 0 <= next_point[1] < self.env.columns:
                    if self.env.using_env[next_point] == 0:
                        if next_point in self.roads_vector1.keys():
                            if self.roads_vector1[next_point][0] == -self.roads_vector1[point][0] and self.roads_vector1[next_point][1] == -self.roads_vector1[point][1]:
                                self.opposite_point[point] = next_point
                        else:
                            break

    def get_marking_points_in_area(self, point1, point2):  # 左上到右下
        for x in range(point2[0] - point1[0] + 1):
            for y in range(point2[1] - point1[1] + 1):
                point = (point1[0] + x, point1[1] + y)
                if point in self.opposite_point.keys() and point not in self.marking_points:
                    self.marking_points.add(point)
                    self.marking_points.add(self.opposite_point[point])

    def single(self, point):
        self.marking_points.add(point)
        self.marking_points.add(self.opposite_point[point])

    def save(self, file_path, file):
        # 保存文件
        np.save(file_path, file)


r = Roads()