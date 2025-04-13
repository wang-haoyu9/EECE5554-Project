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
            parameters=[{'grid_resolution': 0.05,
                         'grid_width': 100,
                         'grid_height': 100,
                         'origin_x': -2.5,
                         'origin_y': -2.5,
                         'ground_threshold': 0.1}],
        ),
        Node(
            package='pointcloud',  # 如果 tsp 节点在同一个包中，否则修改为对应的 package 名称
            executable='tsp_planner',
            name='tsp_planner_node',
            parameters=[
                {'start': '[-0.8, -0.6]'},
                {'goal': '[0.4, 0.5]'},
                {'targets': '[[-0.7, -0.4], [0.0, -0.3]]'}
            ],
            output='screen'
        ),        
    ])