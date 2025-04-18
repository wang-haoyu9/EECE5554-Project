import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped
import math

_liner_velocity_base_ = 5.0 # Line speed base multiplier
_angle_diff_base_ = 2.0  # Angle difference base multiplier

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        self.subscription = self.create_subscription(
            Path,
            '/planned_path',
            self.path_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.current_goal_index = 0
        self.current_path = None
        self.get_logger().info('Robot Control Node has been started.')

    def path_callback(self, msg):
        self.get_logger().info('Received a new path.')
        self.current_path = msg.poses
        self.current_goal_index = 0
        self.move_to_next_goal()

    def move_to_next_goal(self):
        if self.current_path and self.current_goal_index < len(self.current_path):
            goal_pose = self.current_path[self.current_goal_index].pose
            self.get_logger().info(f'Moving to goal {self.current_goal_index + 1}/{len(self.current_path)}.')
            self.navigate_to_goal(goal_pose)
        else:
            self.get_logger().info('Path completed or no path available.')
            self.stop_robot()

    def navigate_to_goal(self, goal_pose):
        # The actual position of the robot. For example, 0.0, 0.0, 0.0.
        robot_x, robot_y, robot_theta = 0.0, 0.0, 0.0
        goal_x, goal_y = goal_pose.position.x, goal_pose.position.y

        # Calculate the angle difference between the target point and the robot
        angle_to_goal = math.atan2(goal_y - robot_y, goal_x - robot_x)
        angle_diff = angle_to_goal - robot_theta

        # Make sure the angle difference is in the range [-pi, pi]
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        # Set linear and angular speed
        twist = Twist()
        twist.linear.x = _liner_velocity_base_  # Linear speed
        twist.angular.z = _angle_diff_base_ * angle_diff  # Angular speed

        self.publisher.publish(twist)

        # Logic for reaching the target point
        distance_to_goal = math.sqrt((goal_x - robot_x) ** 2 + (goal_y - robot_y) ** 2)
        if distance_to_goal < 0.1:  # The tolerance distance of the target point. For example, 10 cm
            self.get_logger().info('Reached the goal.')
            self.current_goal_index += 1
            self.move_to_next_goal()
        else:
            self.get_logger().info(f'Heading to goal: angle_diff={angle_diff:.2f}, distance={distance_to_goal:.2f}')

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher.publish(twist)
        self.get_logger().info('Robot stopped.')


def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()