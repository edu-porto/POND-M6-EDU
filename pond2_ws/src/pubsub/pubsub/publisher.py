import rclpy
from InquirerPy import inquirer
from InquirerPy.utils import InquirerPyKeybindings
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Connecting to a publisher 
from nav_msgs.msg import Odometry

# Handling multiple nodes
from rclpy.executors import MultiThreadedExecutor

# Importing the threading so i can run the cli 
import threading

# Classe que vai receber as informações do webots
class MyWebotsNode(Node):
    def __init__(self):
        super().__init__('webots_node')
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odometry_callback,
            1)
        self.last_odometry_msg = None
        self.last_timestamp = None
        self.last_speed = None

        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
        

    def odometry_callback(self, msg):
        current_time = self.get_clock().now()
        
        if self.last_odometry_msg is not None and self.last_timestamp is not None:
            # Calculate time difference in seconds
            time_delta = (current_time - self.last_timestamp).nanoseconds / 1e9
            
            # Calculate linear speed
            delta_x = msg.pose.pose.position.x - self.last_odometry_msg.pose.pose.position.x
            delta_y = msg.pose.pose.position.y - self.last_odometry_msg.pose.pose.position.y
            linear_speed = (delta_x**2 + delta_y**2)**0.5 / time_delta
            
            # Calculate linear acceleration
            if self.last_speed is not None:
                # Show position and speed information
                self.get_logger().info(f'Linear speed: {linear_speed:.2f} m/s, Positions x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}')
            else:
                self.get_logger().info(f'Robot not moving. Positions x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}')
                
            # Update last speed
            self.last_speed = linear_speed
        else:
            self.get_logger().info('Waiting for more data to calculate speed and acceleration.')
        
        # Update last odometry message and timestamp
        self.last_odometry_msg = msg
        self.last_timestamp = current_time

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
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)
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



# Function that implements the CLI control 
def cli_control(simulated_bot):
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
        execute_cli.spin()
        
    except KeyboardInterrupt:
        pass

def main(args=None):
    rclpy.init(args=args)

    simulated_bot = MoveRobotNode()

    # Create a node for the webots simulation
    webots_node = MyWebotsNode()

    # Create a MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=2)

    # Adding the nodes to the MultiThreaded function 
    executor.add_node(simulated_bot)
    executor.add_node(webots_node)

    # Running the CLI as a thread
    cli_thread = threading.Thread(target=cli_control, args=(simulated_bot,))
    cli_thread.start()
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    

    # rclpy.spin(webots_node)

    simulated_bot.destroy_node()
    webots_node.destroy_node()
    rclpy.shutdown()
    cli_thread.join()

if __name__ == '__main__':
    main()
