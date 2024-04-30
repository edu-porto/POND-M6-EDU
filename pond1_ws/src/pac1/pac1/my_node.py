import rclpy
from rclpy.node import Node
import time
from turtlesim.srv import SetPen, Spawn, Kill
from geometry_msgs.msg import Twist

# Classe do turtlesim
class Turtle(Node):
    def __init__(self):
        super().__init__('turtle')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.timer_ = self.create_timer(1, self.draw_square)
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self.spawn_client = self.create_client(Spawn, 'spawn')
        self.kill_client = self.create_client(Kill, 'kill')
        self.twist_msg_ = Twist()
        self.start_time = time.time()  # Record the start time
    
    def draw_square(self):
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        # Set angular velocity to make the turtle turn
        self.twist_msg_.linear.x = -10.0  # No forward motion
        self.twist_msg_.angular.z = 6.0  # Rotate in plac   

        self.publisher_.publish(self.twist_msg_)

        if elapsed_time > 7:
            self.stop()

    def stop(self):
        # Stop the turtle's motion
        self.twist_msg_.linear.x = 0.0
        self.twist_msg_.angular.y = 0.0
        self.publisher_.publish(self.twist_msg_)
        request = Kill.Request()
        request.name = 'turtle1' 

        self.kill_client.call_async(request)
    
    # def call_kill_service(self):
    #     request = Kill.Request()
    #     request.name = 'turtle1'  # Assuming the turtle's name is 'turtle1'
    #     future = self.kill_client.call_async(request)


def main(args=None):
    rclpy.init(args=args)

    # Criando duas instancias da tartaruga 
    turtle1_controller = Turtle()

    # aqui spawna a tartaruga 
    rclpy.spin(turtle1_controller)

    turtle1_controller.destroy_node()
    # turtle2_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()