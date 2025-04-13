#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import struct
import open3d as o3d

from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2  # ROS2 中可使用此接口
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header

class OccupancyMapNode(Node):
    def __init__(self):
        super().__init__('occupancy_map_node')
        # 参数设置：地图分辨率、宽度、高度以及地图原点（2D地图用 x,y，z 用于描述坐标系）
        self.declare_parameter('grid_resolution', 0.01)  # 单位：米/格
        self.declare_parameter('grid_width', 100)         # 格数
        self.declare_parameter('grid_height', 100)        # 格数
        self.declare_parameter('origin_x', -0.5)           # 地图原点 x 坐标（米）
        self.declare_parameter('origin_y', -0.5)           # 地图原点 y 坐标（米）
        self.declare_parameter('origin_z', 0.0)            # 地图原点 z 坐标（米）
        # 可选：预处理时使用的 z 阈值（用于地面候选过滤）
        self.declare_parameter('ground_threshold', 0.1)    # 例如：0.1 米

        self.grid_resolution = self.get_parameter('grid_resolution').value
        self.grid_width = self.get_parameter('grid_width').value
        self.grid_height = self.get_parameter('grid_height').value
        self.origin_x = self.get_parameter('origin_x').value
        self.origin_y = self.get_parameter('origin_y').value
        self.origin_z = self.get_parameter('origin_z').value
        self.ground_threshold = self.get_parameter('ground_threshold').value

        # 创建 OccupancyGrid 发布器
        self.map_publisher = self.create_publisher(OccupancyGrid, 'map', 10)
        # 订阅点云数据的 topic，这里假设点云消息的 topic 为 "pointcloud"
        self.pointcloud_subscriber = self.create_subscription(
            PointCloud2,
            'pointcloud',
            self.pointcloud_callback,
            10
        )
        self.get_logger().info("occupancy_map_node 启动，订阅 topic 'pointcloud'")

    def pointcloud_callback(self, msg: PointCloud2):
        # 将 PointCloud2 数据读取为点数据列表
        points = list(point_cloud2.read_points(msg, skip_nans=True))
        if not points:
            self.get_logger().warn("收到空的点云数据")
            return

        # 将 list 转换为结构化数组，提取 x, y, z 字段，并转换为标准二维数组，数据类型 float64
        pts_struct = np.array(points)  # dtype类似 [('x', '<f4'), ('y', '<f4'), ('z', '<f4')]
        pts_np = np.stack([pts_struct['x'], pts_struct['y'], pts_struct['z']], axis=1).astype(np.float64)

        # --- 预处理：根据 z 值过滤地面候选点 ---
        # 这里认为 z 值小于 ground_threshold 的点为地面候选，
        # 你可以根据实际情况调整这个阈值
        candidate_indices = np.where(pts_np[:, 2] < self.ground_threshold)[0]
        if candidate_indices.size < 50:
            self.get_logger().warn("预处理后的地面候选点数量不足，使用全部点进行分割")
            pts_for_segmentation = pts_np
        else:
            pts_for_segmentation = pts_np[candidate_indices]
        
        # 使用 Open3D 构造点云对象用于平面分割
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts_for_segmentation)
        
        # 使用 RANSAC 分割检测平面（假设地面为最大的平面）
        plane_model, inliers = pcd.segment_plane(distance_threshold=0.05,
                                                 ransac_n=3,
                                                 num_iterations=1000)
        [a, b, c, d] = plane_model
        self.get_logger().info("检测到平面: {:.3f}x + {:.3f}y + {:.3f}z + {:.3f} = 0".format(a, b, c, d))
        
        # 计算旋转矩阵：将检测到的平面法向量归一化，并旋转到 [0, 0, 1]
        normal = np.array([a, b, c])
        normal_norm = np.linalg.norm(normal)
        if normal_norm < 1e-6:
            self.get_logger().warn("检测到的平面法向量异常")
            R = np.eye(3)
        else:
            normal = normal / normal_norm
            target = np.array([0, 0, 1])
            v = np.cross(normal, target)
            s = np.linalg.norm(v)
            if s < 1e-6:
                R = np.eye(3)
            else:
                c_val = np.dot(normal, target)
                vx = np.array([[0, -v[2], v[1]],
                               [v[2], 0, -v[0]],
                               [-v[1], v[0], 0]])
                R = np.eye(3) + vx + np.dot(vx, vx) * ((1 - c_val) / (s ** 2))
        
        # 对所有原始点进行旋转，使平面对齐到 z=0
        pts_rotated = np.dot(pts_np, R.T)
        
        # 利用检测到的内点，在旋转后的点云中计算内点 z 坐标的平均值
        inlier_points = pts_for_segmentation[inliers]
        inlier_rotated = np.dot(inlier_points, R.T)
        mean_inlier = np.mean(inlier_rotated, axis=0)
        # 计算平移量，将平均 z 坐标调整至 0
        T = np.array([0, 0, -mean_inlier[2]])
        pts_aligned = pts_rotated + T
        
        # --- 构造 OccupancyGrid ---
        # 这里只使用对齐后点的 x 和 y 坐标生成 2D 地图
        # 初始化 occupancy_array 为未知区域（例如 -1 代表未知）
        occupancy_array = -1 * np.ones((self.grid_height, self.grid_width), dtype=np.int8)

        # 定义一个高度阈值，低于该值认为是地面；高于该值认为是障碍物
        # 请根据实际情况调整这个阈值，例如 0.1 米
        obstacle_threshold = 0.1

        for p in pts_aligned:
            x, y, z = p[0], p[1], p[2]
            grid_x = int((x - self.origin_x) / self.grid_resolution)
            grid_y = int((y - self.origin_y) / self.grid_resolution)
            if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height:
                if z > obstacle_threshold:
                    # z 值高于阈值，认为这是障碍物
                    occupancy_array[grid_y, grid_x] = 100
                else:
                    # z 值低于或等于阈值，认为这是地面
                    occupancy_array[grid_y, grid_x] = 10

        occ_grid_msg = self.generate_occupancy_grid_msg(occupancy_array)
        self.map_publisher.publish(occ_grid_msg)
        self.get_logger().info("发布新的 occupancy map")

    def generate_occupancy_grid_msg(self, occupancy_array: np.ndarray) -> OccupancyGrid:
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        info = MapMetaData()
        info.map_load_time = self.get_clock().now().to_msg()
        info.resolution = self.grid_resolution
        info.width = self.grid_width
        info.height = self.grid_height
        info.origin.position.x = self.origin_x
        info.origin.position.y = self.origin_y
        info.origin.position.z = self.origin_z
        info.origin.orientation.x = 0.0
        info.origin.orientation.y = 0.0
        info.origin.orientation.z = 0.0
        info.origin.orientation.w = 1.0
        msg.info = info

        msg.data = occupancy_array.flatten().tolist()
        return msg

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

# Run with ROS2 launch (Don't forget to source install/setup.bash):
# ros2 launch pointcloud pc_launch.py ply:="/home/shuo/EECE5554-Project/data/box1_high density.ply" rotation:="90,0,0"