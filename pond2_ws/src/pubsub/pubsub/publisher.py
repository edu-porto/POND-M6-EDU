import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class MoveRobotNode(Node):
    def __init__(self):
        super().__init__('move_robot_node')

        # Publisher to the velocity command topic
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber to the current velocity topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.velocity_callback,
            10)

        self.get_logger().info('MoveRobotNode has been started.')

    def move_forward(self, duration):
        # Create a Twist message with forward velocity
        msg = Twist()
        msg.linear.x = 0.5  # Adjust the speed as necessary
        msg.angular.z = 0.0

        self.get_logger().info('Moving forward.')

        start_time = time.time()
        while (time.time() - start_time) < duration:
            self.publisher_.publish(msg)
            self.get_logger().info('Published forward velocity command.')
            self.get_logger().info('Waiting for current velocity...')
            rclpy.spin_once(self)
        
        # Stop the robot after moving forward
        msg.linear.x = 0.0
        self.publisher_.publish(msg)
        self.get_logger().info('Stopped moving.')

    def velocity_callback(self, msg):
        self.get_logger().info(f'Received current velocity: Linear.x = {msg.linear.x}, Angular.z = {msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)

    move_robot_node = MoveRobotNode()

    try:
        move_robot_node.move_forward(5000)  # Move forward for 5 seconds
    except KeyboardInterrupt:
        pass

    move_robot_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
