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
        )
    ])