import rclpy
from rclpy.node import Node
import time
from turtlesim.srv import SetPen, Spawn, Kill
from geometry_msgs.msg import Twist

# Classe do turtlesim
class Turtle(Node):

    # Definindo spawn, kill, timer e o tamanho do desenho
    def __init__(self):
        super().__init__('turtle')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.timer_ = self.create_timer(1, self.draw_square)
        self.pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        self.spawn_client = self.create_client(Spawn, 'spawn')
        self.kill_client = self.create_client(Kill, 'kill')
        self.twist_msg_ = Twist()
        self.start_time = time.time()  
        self.color_changed = False
        self.side_length = 1.0  


    # Aqui é a função que desenha o circulo 
    def draw_square(self):
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        self.twist_msg_.linear.x = self.side_length*0.7
        self.twist_msg_.angular.z = 1.5  
        self.publisher_.publish(self.twist_msg_)

        #time.sleep(1)  

        self.side_length += 1

        # Se o tempo passado é maior que 10 mata a tartaruga 
        if elapsed_time > 10:
            self.stop()
        
        # Definindo as cores com base no tempo
        elif elapsed_time <= 2:
            self.blue()
        elif elapsed_time <= 4:
            self.green()
        elif elapsed_time <= 6:
            self.greener()
        elif elapsed_time <= 8:
            self.even_greener()
        elif elapsed_time <= 10:
            self.pale()

    
    # definindo cores
    def green(self):
        request = SetPen.Request()
        request.r = 56  # Red color
        request.g = 102    # Green color
        request.b = 65    # Blue color
        request.width = 4

        future = self.pen_client.call_async(request)

    def greener(self):
        request = SetPen.Request()
        request.r = 106  # Red color
        request.g = 153    # Green color
        request.b = 78    # Blue color
        request.width = 6

        future = self.pen_client.call_async(request)

    def even_greener(self):
        request = SetPen.Request()
        request.r = 167  # Red color
        request.g = 201    # Green color
        request.b = 87    # Blue color
        request.width = 8

        future = self.pen_client.call_async(request)

    def blue(self):
        request = SetPen.Request()
        request.r = 21  # Red color
        request.g = 50    # Green color
        request.b = 67    # Blue color
        request.width = 2

        future = self.pen_client.call_async(request)
    
    def pale(self):
        request = SetPen.Request()
        request.r = 242  # Red color
        request.g = 232   # Green color
        request.b = 207    # Blue color
        request.width = 10

        future = self.pen_client.call_async(request)
    

    def stop(self):
        # Stop the turtle's motion
        self.twist_msg_.linear.x = 0.0
        self.twist_msg_.angular.z = 0.0
        self.publisher_.publish(self.twist_msg_)
        request = Kill.Request()
        request.name = 'turtle1' 

        self.kill_client.call_async(request)


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