#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import json
import math
from queue import PriorityQueue

from nav_msgs.msg import OccupancyGrid, MapMetaData, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

# A*算法的基本实现
def a_star_search(grid, start, goal, obstacle_threshold=50):
    """
    在二维栅格地图上利用A*算法搜索从start到goal的路径。
    grid: 2D numpy array, 其中值大于 obstacle_threshold 的格子视为障碍物
    start, goal: (row, col) 格式的起始和目标索引
    返回：(path, cost)
      path: 从start到goal的格子索引列表，如 [(r1,c1), (r2,c2), ...]
      cost: 路径代价（整数）
    """
    # 检查起点和终点是否可通行
    if grid[start[0], start[1]] > obstacle_threshold or grid[goal[0], goal[1]] > obstacle_threshold:
        return None, float('inf')

    open_set = PriorityQueue()
    open_set.put((0, start))
    came_from = {}
    g_score = {start: 0}

    # 使用曼哈顿距离作为启发式函数
    def heuristic(a, b):
        return abs(a[0]-b[0]) + abs(a[1]-b[1])

    while not open_set.empty():
        current_priority, current = open_set.get()
        if current == goal:
            # 构造路径
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path, g_score[goal]

        # 获取邻近格子（四邻域或八邻域，这里采用八邻域）
        for dr in [-1, 0, 1]:
            for dc in [-1, 0, 1]:
                if dr == 0 and dc == 0:
                    continue
                neighbor = (current[0] + dr, current[1] + dc)
                # 检查是否在地图范围内
                if 0 <= neighbor[0] < grid.shape[0] and 0 <= neighbor[1] < grid.shape[1]:
                    # 如果是障碍物则跳过
                    if grid[neighbor[0], neighbor[1]] > obstacle_threshold:
                        continue
                    # 采用对角线移动代价为1.4，否则为1.0
                    if dr != 0 and dc != 0:
                        tentative_cost = g_score[current] + 1.4
                    else:
                        tentative_cost = g_score[current] + 1.0
                    if neighbor not in g_score or tentative_cost < g_score[neighbor]:
                        came_from[neighbor] = current
                        g_score[neighbor] = tentative_cost
                        f = tentative_cost + heuristic(neighbor, goal)
                        open_set.put((f, neighbor))
    return None, float('inf')


def reconstruct_path(came_from, current):
    """辅助函数：根据 came_from 字典构造路径"""
    path = [current]
    while current in came_from:
        current = came_from[current]
        path.append(current)
    return path[::-1]


# 简单的贪心TSP算法：从起点出发，每次选距离当前最近的未访问目标，最后到达终点
def tsp_greedy(distance_matrix):
    N = len(distance_matrix)
    # 假设：第0个是起点，最后一个是终点，中间是目标物品
    unvisited = set(range(1, N-1))
    route = [0]
    current = 0
    while unvisited:
        next_node = min(unvisited, key=lambda x: distance_matrix[current][x])
        route.append(next_node)
        unvisited.remove(next_node)
        current = next_node
    route.append(N-1)
    return route


class TSPPlannerNode(Node):
    def __init__(self):
        super().__init__('tsp_planner_node')
        # 订阅已有的OccupancyGrid地图
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10
        )
        # 发布最终规划路径（nav_msgs/Path）
        self.path_publisher = self.create_publisher(Path, 'planned_path', 10)

        # 用于存储OccupancyGrid数据（当收到第一个地图时触发规划）
        self.occupancy_grid = None

        # 假设在 TSP 节点的 __init__ 中声明参数（可能已经声明了，如下）：
        self.declare_parameter('start', '[-0.8, -0.6]')
        self.declare_parameter('goal', '[0.4, 0.5]')
        self.declare_parameter('targets', '[[-0.7, -0.4], [0.0, -0.3]]')

        # 获取并解析参数
        start_str = self.get_parameter('start').value
        goal_str = self.get_parameter('goal').value
        targets_str = self.get_parameter('targets').value

        try:
            # 使用 json.loads 解析字符串，转换为列表
            self.start = json.loads(start_str)
            self.goal = json.loads(goal_str)
            self.targets = json.loads(targets_str)
            self.get_logger().info(
                f"Parsed parameters: start {self.start}, goal {self.goal}, targets {self.targets}"
            )
        except Exception as e:
            self.get_logger().error(f"解析参数失败: {e}")
        # 设置默认值
            self.start = [-0.8, -0.6]
            self.goal = [0.4, 0.5]
            self.targets = [[-0.7, -0.4], [0.0, -0.3]]
        
    def map_callback(self, msg: OccupancyGrid):
        self.get_logger().info("收到 OccupancyGrid, 开始TSP规划")
        # 转换 OccupancyGrid 消息为二维数组
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data).reshape((height, width))
        self.occupancy_grid = data

        # 等待一定时间保证OccupancyGrid数据稳定后规划（这里直接调用）
        self.plan_path(msg.info)

    def world_to_grid(self, pos, info):
        # pos: [x, y] (世界坐标)
        # info: MapMetaData
        # 将世界坐标转换为栅格索引（行,列）
        col = int((pos[0] - info.origin.position.x) / info.resolution)
        row = int((pos[1] - info.origin.position.y) / info.resolution)
        # 注意数组索引(row, col)对应 (y, x)
        return (row, col)

    def grid_to_world(self, grid_idx, info):
        # grid_idx: (row, col)
        x = info.origin.position.x + (grid_idx[1] + 0.5) * info.resolution
        y = info.origin.position.y + (grid_idx[0] + 0.5) * info.resolution
        return [x, y]

    def plan_path(self, info):
        # 将用户指定的起点、目标点、终点转换为栅格索引
        start_idx = self.world_to_grid(self.start, info)
        goal_idx = self.world_to_grid(self.goal, info)
        target_indices = [self.world_to_grid(t, info) for t in self.targets]

        # 构建节点列表：起点、所有目标点、终点
        nodes = [start_idx] + target_indices + [goal_idx]
        N = len(nodes)
        distance_matrix = np.zeros((N, N))
        path_storage = {}

        # 对于每对节点计算局部路径和代价
        for i in range(N):
            for j in range(i+1, N):
                path, cost = a_star_search(self.occupancy_grid, nodes[i], nodes[j])
                distance_matrix[i, j] = cost
                distance_matrix[j, i] = cost
                path_storage[(i, j)] = path
                path_storage[(j, i)] = list(reversed(path)) if path is not None else None

        self.get_logger().info("距离矩阵:\n" + str(distance_matrix))
        route = tsp_greedy(distance_matrix)
        self.get_logger().info("TSP规划顺序: " + str(route))

        # 拼接完整路径
        full_path_grid = []
        for i in range(len(route)-1):
            seg = path_storage.get((route[i], route[i+1]))
            if seg is None:
                self.get_logger().warn("两节点间无路径: {} 到 {}".format(route[i], route[i+1]))
                continue
            # 拼接时，避免重复添加节点
            if full_path_grid:
                full_path_grid.extend(seg[1:])
            else:
                full_path_grid.extend(seg)

        # 将栅格索引转换为世界坐标点
        full_path_world = [self.grid_to_world(idx, info) for idx in full_path_grid]

        # 发布路径为 nav_msgs/Path 消息
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        for pt in full_path_world:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = pt[0]
            pose.pose.position.y = pt[1]
            pose.pose.position.z = 0.0  # 2D地图 z=0
            # 默认姿态
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)
        self.get_logger().info("发布规划路径，路径节点数量: {}".format(len(path_msg.poses)))

def main(args=None):
    rclpy.init(args=args)
    node = TSPPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
