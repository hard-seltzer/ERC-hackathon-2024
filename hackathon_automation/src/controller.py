import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray, Pose
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math

class TurtlebotController(Node):
    def __init__(self):
        super().__init__('tb3_controller')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_sub = self.create_subscription(PoseArray, '/planned_path', self.path_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.path = []
        self.current_pose = None
        self.goal_index = 0
        self.timer = self.create_timer(0.1, self.control_loop)

    def path_callback(self, msg):
        self.path = msg.poses
        self.get_logger().info('Received planned path')

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    def control_loop(self):
        if not self.path or self.current_pose is None:
            return

        if self.goal_index >= len(self.path):
            self.get_logger().info('Path completed')
            return

        goal = self.path[self.goal_index]
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        goal_x = goal.position.x
        goal_y = goal.position.y

        distance = math.sqrt((goal_x - current_x)**2 + (goal_y - current_y)**2)

        if distance < 0.1:
            self.goal_index += 1
            return

        angle = math.atan2(goal_y - current_y, goal_x - current_x)
        _, _, current_yaw = euler_from_quaternion([self.current_pose.orientation.x, self.current_pose.orientation.y,
                                                   self.current_pose.orientation.z, self.current_pose.orientation.w])

        angle_diff = angle - current_yaw
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        elif angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        cmd_vel = Twist()
        if abs(angle_diff) > 0.1:
            cmd_vel.angular.z = 0.3 if angle_diff > 0 else -0.3
        else:
            cmd_vel.linear.x = 0.2

        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    controller = TurtlebotController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()