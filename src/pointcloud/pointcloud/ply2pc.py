# pip install open3d
import open3d as o3d
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from rclpy.logging import get_logger

_logger = get_logger('pointcloud')

def create_point_cloud2(points) -> PointCloud2:
    """Convert point cloud data to a PointCloud2 message."""
    msg = PointCloud2()
    msg.header.frame_id = "map"
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12  # 3 * 4 bytes for float32
    msg.row_step = msg.point_step * len(points)
    msg.is_dense = True
    msg.data = np.asarray(points, dtype=np.float32).tobytes()
    return msg

def ply_to_pointcloud2(file_path: str, rotation: tuple[float,float,float]) -> PointCloud2:
    """Read a .ply file and convert it to a PointCloud2 message."""
    # Load the .ply file using Open3D
    pcd = o3d.io.read_point_cloud(file_path)

    # Convert rotation from degrees to radians
    rotation_rad = np.radians(rotation)
    
    # Create rotation matrices for x, y, and z axes
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(rotation_rad[0]), -np.sin(rotation_rad[0])],
        [0, np.sin(rotation_rad[0]), np.cos(rotation_rad[0])]
    ])
    Ry = np.array([
        [np.cos(rotation_rad[1]), 0, np.sin(rotation_rad[1])],
        [0, 1, 0],
        [-np.sin(rotation_rad[1]), 0, np.cos(rotation_rad[1])]
    ])
    Rz = np.array([
        [np.cos(rotation_rad[2]), -np.sin(rotation_rad[2]), 0],
        [np.sin(rotation_rad[2]), np.cos(rotation_rad[2]), 0],
        [0, 0, 1]
    ])
    
    # Combine the rotation matrices
    R = Rz @ Ry @ Rx
    
    # Extract points as a numpy array and apply the rotation
    points = np.asarray(pcd.points) @ R.T
    return create_point_cloud2(points)

