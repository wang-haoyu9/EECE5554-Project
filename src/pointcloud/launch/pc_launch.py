from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter

def generate_launch_description():
    ply_path = LaunchConfiguration('ply')
    rotation = LaunchConfiguration('rotation')

    return LaunchDescription([
        SetParameter(name='ply', value=ply_path),
        SetParameter(name='rotation', value=rotation),
        Node(
            package='pointcloud',
            executable='pc_node',
            name='pointcloud_node',
            parameters=[{'ply': ply_path},
                        {'rotation': rotation}],
        ),

        Node(
            package='pointcloud',  # 如果 occupancy map 节点在同一个包中，否则修改为对应的 package 名称
            executable='occupancy_map_node',  # 对应的 executable 名称
            name='occupancy_map_node',
            parameters=[{'grid_resolution': 0.005,
                         'grid_width': 1000,
                         'grid_height': 1000,
                         'origin_x': -2.5,
                         'origin_y': -2.5,
                         'ground_threshold': 0.1}],
        ),
        Node(
            package='pointcloud',  # 如果 tsp 节点在同一个包中，否则修改为对应的 package 名称
            executable='tsp_planner',
            name='tsp_planner_node',
            output='screen'
        ),        
    ])