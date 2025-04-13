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
        # 参数设置：地图分辨率、宽度、高度以及地图原点（2D地图用 x,y，z 仅用于描述平面高度）
        self.declare_parameter('grid_resolution', 0.1)  # 单位：米/格
        self.declare_parameter('grid_width', 100)         # 格数
        self.declare_parameter('grid_height', 100)        # 格数
        self.declare_parameter('origin_x', -5.0)           # 地图原点 x 坐标（米）
        self.declare_parameter('origin_y', -5.0)           # 地图原点 y 坐标（米）
        self.declare_parameter('origin_z', 0.0)            # 地图原点 z 坐标（米），用于描述坐标系，生成地图时不用修改

        self.grid_resolution = self.get_parameter('grid_resolution').value
        self.grid_width = self.get_parameter('grid_width').value
        self.grid_height = self.get_parameter('grid_height').value
        self.origin_x = self.get_parameter('origin_x').value
        self.origin_y = self.get_parameter('origin_y').value
        self.origin_z = self.get_parameter('origin_z').value

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
        pts_np = np.asarray(points)

        # 如果只有一个点，pts_np 可能是一维数组，扩展为二维数组
        if pts_np.ndim == 1:
            pts_np = pts_np.reshape(1, -1)

        pts_np = pts_np[:, :3]

        # 转换数据类型为 float64
        pts_np = np.array(points)  # 构造结构化数组，dtype 类似 [('x', '<f4'), ('y', '<f4'), ('z', '<f4')]
        pts_np = np.stack([pts_np['x'], pts_np['y'], pts_np['z']], axis=1).astype(np.float64)

        # 使用 Open3D 创建点云对象
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pts_np)
        
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
        
        # 对所有点进行旋转，将平面对齐到 z = 0 的水平面
        pts_rotated = np.dot(pts_np, R.T)
        
        # 计算内点在旋转后坐标的平均值，用于平移调整（使地面处于 z=0）
        inlier_points = pts_np[inliers]
        inlier_rotated = np.dot(inlier_points, R.T)
        mean_inlier = np.mean(inlier_rotated, axis=0)
        T = np.array([0, 0, -mean_inlier[2]])
        pts_aligned = pts_rotated + T
        
        # 构造 2D OccupancyGrid，使用对齐后的 x 和 y 坐标，
        # 这里不考虑 z 轴高度（假设经过处理后，地面 z≈0）
        occupancy_array = np.zeros((self.grid_height, self.grid_width), dtype=np.int8)
        for p in pts_aligned:
            x, y, z = p[0], p[1], p[2]
            # 这里可根据需要设置额外的条件，比如只考虑高于某个阈值的点为障碍物
            # 此处直接对所有点进行投影，标记为占据
            grid_x = int((x - self.origin_x) / self.grid_resolution)
            grid_y = int((y - self.origin_y) / self.grid_resolution)
            if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_height:
                occupancy_array[grid_y, grid_x] = 100
        
        # 将生成的 occupancy_array 转换为 OccupancyGrid 消息并发布
        occ_grid_msg = self.generate_occupancy_grid_msg(occupancy_array)
        self.map_publisher.publish(occ_grid_msg)
        self.get_logger().info("发布新的 occupancy map")

    def generate_occupancy_grid_msg(self, occupancy_array: np.ndarray) -> OccupancyGrid:
        msg = OccupancyGrid()
        # 填充 header 信息
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        # 设置地图元数据
        info = MapMetaData()
        info.map_load_time = self.get_clock().now().to_msg()
        info.resolution = self.grid_resolution
        info.width = self.grid_width
        info.height = self.grid_height
        # 地图原点的位姿信息
        info.origin.position.x = self.origin_x
        info.origin.position.y = self.origin_y
        info.origin.position.z = self.origin_z
        info.origin.orientation.x = 0.0
        info.origin.orientation.y = 0.0
        info.origin.orientation.z = 0.0
        info.origin.orientation.w = 1.0
        msg.info = info

        # 将二维数组展平成一维列表
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
