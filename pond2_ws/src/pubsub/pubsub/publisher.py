import rclpy
from InquirerPy import inquirer
from InquirerPy.utils import InquirerPyKeybindings
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


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

        self.get_logger().info('O robô foi iniciado .')
        
    def move_forward(self):
        # Create a Twist message with forward velocity
        msg = Twist()
        msg.linear.x = 0.2  # Adjust the speed as necessary
        msg.angular.z = 0.0

        self.get_logger().info('Moving forward.')

        # start_time = time.time()
        # Aqui é publicada a msg pro robo ir pra frente
        # while (time.time() - start_time) < duration:
        self.publisher_.publish(msg)
        # rclpy.spin_once(self)
        
        # # Stop the robot after moving forward
        # msg.linear.x = 0.0
        # self.publisher_.publish(msg)
        # self.get_logger().info('Stopped moving.')

    def turn_left(self):
        msg = Twist()
        msg.angular.z = 0.4

        self.get_logger().info('Moving to the left')
        self.publisher_.publish(msg)

    def turn_right(self):
        msg = Twist()
        msg.angular.z = -0.4

        self.get_logger().info('Moving to the right')
        self.publisher_.publish(msg)

    def backward(self):
        msg = Twist()
        msg.linear.x  = -0.2

        self.get_logger().info('Backwars')
        self.publisher_.publish(msg)

    def brake(self):
        msg = Twist()
        msg.linear.x  = 0.0

        self.get_logger().info('Braking')
        self.publisher_.publish(msg)


    def emergency(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

    def velocity_callback(self, msg):
        self.get_logger().info(f'Status do robô: Linear.x = {msg.linear.x}, Angular.z = {msg.angular.z}')


def main(args=None):
    rclpy.init(args=args)

    simulated_bot = MoveRobotNode()
    
    cli = inquirer.text(message="")
    keybindings: InquirerPyKeybindings = {
            "interrupt": [{"key": "q"}, {"key": "c-c"}],
        }
    
    @cli.register_kb("w")
    def forward(_):
        simulated_bot.move_forward()
        print("Moving forward")

    @cli.register_kb("a")
    def left(_):
        simulated_bot.turn_left()
        print("Turning left")
    
    @cli.register_kb("d")
    def right(_):
        simulated_bot.turn_right()
        print("Turning right") 

    @cli.register_kb("s")
    def backwards(_):
        simulated_bot.backward()
        print("Moving backwards")

    @cli.register_kb("space")
    def brake(_):
        simulated_bot.brake()
        print("Braking")    
        
    @cli.register_kb("q")
    def emergency_stop(_):
        simulated_bot.emergency()
        print("Emergency stop")

    execute_cli = cli.execute()

    try:
      execute_cli
    except KeyboardInterrupt:
        pass

    simulated_bot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
