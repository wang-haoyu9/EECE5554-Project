import os
import sys
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from std_msgs.msg import Header
# https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html
from sensor_msgs.msg import PointCloud2

from pointcloud.ply2pc import ply_to_pointcloud2

_logger = get_logger('pointcloud')

class PointCloudNode(Node):
    """Class Node for PointCloud2"""

    global _logger

    def __init__(self):
        """
        The initialization function does the following:
            All ROS2 node initializing stuff 
            Set ROS2 node parameter "ply", "rotation"
            Read the PLY file from "ply" and convert it to a PointCloud2 message
            Create ROS2 publisher for topic "/pointcloud" using customized message "PointCloud2"
            Other settings for ROS2 publisher
        """
        super().__init__('pointcloud_node')

        self.declare_parameter('ply', 'data/box1_high density.ply')
        self.declare_parameter('rotation', '0,0,0')
        self.ply_path = self.get_parameter('ply').value
        # rotation is a tuple of 3 floats (x, y, z)
        self.rotation = tuple(map(float, self.get_parameter('rotation').value.split(',')))
        _logger.info(f"PLY file path: {self.ply_path}") # DEBUG

        if not self.path_check():
            self.stop_node()

        self.publisher_ = self.create_publisher(PointCloud2, 'pointcloud', 10)
        
        # Publish the first message immediately after initialization
        self.timer_callback()

        timer_period = 300.0 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def path_check(self: str) -> bool:
        """Check if the given path is valid.
        
        Returns:
            bool: True if the path is valid, False otherwise.
        """
        if not os.path.exists(self.ply_path):
            _logger.fatal(f"Path {self.ply_path} does not exist. A absolute path is recommended.")
            return False
        return True

    def stop_node(self):
        """Stop the node and exit the program."""
        _logger.info('Shutting down the node...')
        rclpy.shutdown()
        sys.exit(0)  # Exit the program cleanly

    def timer_callback(self):
        """Callback for ROS2 publisher

        Simply call make_pub_msg() to make the content for the cutomized message 
        and then passes to publisher.
        """
        msg = self.make_pub_msg()
        # msg can be None or empty
        if msg:
            self.publisher_.publish(msg)
            _logger.info('Published PointCloud2 message.')

    def make_pub_msg(self) -> PointCloud2:
        """Making the customized message to the topic.
        
        Returns:
            PointCloud2: The PointCloud2 message.
        """
        message = PointCloud2()
        message = ply_to_pointcloud2(self.ply_path, self.rotation)
        return message


# Run with ROS2 launch (Don't forget to source install/setup.bash):
#   ros2 launch pointcloud pc_launch.py ply:="/home/why/Documents/EECE5554-Project/data/box1_high density.ply" rotation:="90,0,0"
def main(args=None):
    """Main executable for gps_driver.driver
    
    Parameter:
        ply: PLY file path. An absolute path is recommended.
        rotation: Rotation angles in degrees (x, y, z). Default is 0,0,0.
    """
    # Node program
    rclpy.init(args=args)

    node = PointCloudNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
