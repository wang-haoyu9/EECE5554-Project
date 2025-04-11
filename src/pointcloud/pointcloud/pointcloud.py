import os
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from std_msgs.msg import Header
# https://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/PointCloud2.html
from sensor_msgs.msg import PointCloud2

from pointcloud.ply2pc import ply_to_pointcloud2

_logger = get_logger('pointcloud')

class PointCloudNode(Node):
    """Class Driver for GPS driver
    """

    global _logger

    def __init__(self):
        """
        The initialization function does the following:
            All ROS2 node initializing stuff 
            Set ROS2 node parameter "port" and set default "/dev/ttyUSB0"
            Create ROS2 publisher for topic "/topic" using customized message "GPSmsg"
            Open serial port with "port" and set Baud Rate 4800
            Other settings for ROS2 publisher
        """
        super().__init__('pointcloud_node')

        self.declare_parameter('ply', 'data/coke_can.ply')
        self.ply_path = self.get_parameter('ply').value
        _logger.info(f"PLY file path: {self.ply_path}") # DEBUG

        if not self.path_check():
            raise FileNotFoundError

        self.publisher_ = self.create_publisher(PointCloud2, 'pointcloud', 10)
        timer_period = 60.0 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def path_check(self):
        """Check if the given path is valid.
        
        Returns:
            bool: True if the path is valid, False otherwise.
        """
        if not os.path.exists(self.ply_path):
            _logger.fatal(f"Path {self.ply_path} does not exist.")
            return False
        return True

    def timer_callback(self):
        """Callback for ROS2 publisher

        Simply call make_pub_msg() to make the content for the cutomized message 
        and then passes to publisher.
        """
        msg = self.make_pub_msg()
        # msg can be None or empty
        if msg:
            self.publisher_.publish(msg)

    def make_pub_msg(self):
        """Making the customized message to the topic.
        
        Returns:
            GPSmsg: customized message :class:`GPSmsg` after being parsed 
            from GPGGA string read from serial port, or None if not a GPGGA 
            string or error occurs.
        """
        message = PointCloud2()
        message = ply_to_pointcloud2(self.ply_path)
        return message


# Run with ROS2 launch (Don't forget to source install/setup.bash):
#   ros2 launch pointcloud pc_launch.py ply:="/home/why/Documents/EECE5554-Project/data/coke_can.ply"
def main(args=None):
    """Main executable for gps_driver.driver
    
    Parameter:
        port: serial port address. Default '/dev/ttyUSB0'.
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
