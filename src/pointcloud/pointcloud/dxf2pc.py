# pip install ezdxf
import ezdxf
import numpy as np
# https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html
from sensor_msgs.msg import PointCloud2, PointField
from rclpy.logging import get_logger

_logger = get_logger('dxf2pc')

import rclpy
import std_msgs.msg

def dxf_to_pointcloud2(dxf_filepath: str, frame_id: str = "map") -> PointCloud2:
    """
    将DXF文件中的3D实体转换为ROS2 PointCloud2消息。

    此函数从DXF文件中提取3D点，并将它们组织成ROS2 PointCloud2消息。
    它处理DXF文件中的LINE、POINT和3DFACE实体。  其他实体类型将被忽略。

    Args:
        dxf_filepath (str): 要读取的DXF文件的路径。
        frame_id (str, optional): PointCloud2消息的frame_id。 默认为 "map"。

    Returns:
        sensor_msgs.msg.PointCloud2: 包含从DXF文件提取的点的PointCloud2消息。

    Raises:
        Exception: 如果DXF文件无法打开或解析，则引发异常。
    """
    try:
        doc = ezdxf.readfile(dxf_filepath)  # 使用 ezdxf 读取 DXF 文件
    except Exception as e:
        raise Exception(f"无法读取DXF文件 {dxf_filepath}: {e}")

    msp = doc.modelspace()  # 获取模型空间以访问实体
    points = []

    # 遍历DXF文件中的所有实体
    for entity in msp:
        if entity.dxftype() == 'LINE':
            # 获取线段的起点和终点
            start_point = entity.dxf.start
            end_point = entity.dxf.end
            points.append(start_point)
            points.append(end_point)
        elif entity.dxftype() == 'POINT':
            # 获取点的坐标
            point = entity.dxf.location
            points.append(point)
        elif entity.dxftype() == '3DFACE':
            # 获取3DFACE的顶点。
            v1 = entity.dxf.vtx0
            v2 = entity.dxf.vtx1
            v3 = entity.dxf.vtx2
            v4 = entity.dxf.vtx3  # 3DFACE可能有第四个顶点

            points.append(v1)
            points.append(v2)
            points.append(v3)
            if v4 != v3:  # 避免重复点，如果v4与v3相同
                points.append(v4)
        # 忽略其他实体类型
    if not points:
        print(f"DXF 文件中没有找到任何点，线或3DFace: {dxf_filepath}")
        return None

    # 将点转换为NumPy数组
    np_points = np.array(points, dtype=np.float32)
    # 重塑数组以匹配PointCloud2结构 [数量, 字段数]
    np_points = np_points.reshape(-1, 3)

    # 定义PointCloud2字段
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]

    # 创建PointCloud2消息
    header = std_msgs.msg.Header()
    header.stamp = rclpy.time.Time().to_msg() # 使用rclpy.time
    header.frame_id = frame_id

    point_step = 12  # 每个点的大小 (3个float32值)
    row_step = point_step * np_points.shape[0]  # 每行的大小
    data = np_points.tobytes()  # 将NumPy数组转换为字节

    pointcloud_msg = PointCloud2(
        header=header,
        height=1,  # 无序点云的高度为1
        width=np_points.shape[0],  # 点的数量
        fields=fields,
        is_bigendian=False,  # 字节顺序
        point_step=point_step,
        row_step=row_step,
        data=data,
        is_dense=True,  # 所有点都是有效的
    )
    return pointcloud_msg

def main(args=None):
     pass

if __name__ == '__main__':
    main()
