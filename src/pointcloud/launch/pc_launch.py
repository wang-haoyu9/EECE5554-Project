from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetParameter

def generate_launch_description():
    dxf_path = LaunchConfiguration('dxf')

    return LaunchDescription([
        SetParameter(name='dxf', value=dxf_path),
        Node(
            package='pointcloud',
            executable='pc_node',
            name='pointcloud_node',
            parameters=[{'dxf': dxf_path}]
        )
    ])