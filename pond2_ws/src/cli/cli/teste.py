from InquirerPy import inquirer
from InquirerPy.utils import patched_print as print
from InquirerPy.utils import InquirerPyKeybindings
from rclpy.node import Node
import time
from geometry_msgs.msg import Twist, Vector3

class TurtleBot(Node):
    def __init__(self):
        super().__init__('turtlebot')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

    def move(self, linear: Vector3, angular: Vector3, duration: float):
        msg = Twist()
        msg.linear = linear
        msg.angular = angular
        self.publisher_.publish(msg)
        time.sleep(duration)
        self.stop()

    def stop(self):
        stop_msg = Twist()
        self.publisher_.publish(stop_msg)

    def forward(self, speed: float, duration: float):
        self.move(Vector3(x=speed, y=0.0, z=0.0), Vector3(), duration)

    def backward(self, speed: float, duration: float):
        self.move(Vector3(x=-speed, y=0.0, z=0.0), Vector3(), duration)

    def left(self, speed: float, duration: float):
        self.move(Vector3(), Vector3(z=speed), duration)

    def right(self, speed: float, duration: float):
        self.move(Vector3(), Vector3(z=-speed), duration)

    def emergency_stop(self):
        self.move(Vector3(x=0.0, y=0.0, z=0.0), Vector3(), 0.0)


def main():
    name_prompt = inquirer.text(message="")
    keybindings: InquirerPyKeybindings = {
            "interrupt": [{"key": "q"}, {"key": "c-c"}],
        }
    
    robot = TurtleBot()

    
    @name_prompt.register_kb("w")
    def forward(_):
        robot.move_forward(0.1, 1.0)
        print("Hello W")

    @name_prompt.register_kb("s")
    def backwards(_):
        print("Hello S")

    @name_prompt.register_kb("a")
    def left(_):
        print("Hello A")

    @name_prompt.register_kb("d")
    def right(_):
        print("Hello D")

    name = name_prompt.execute()
    kb_activate = True

    try:
      return name_prompt( keybindings=keybindings)
    except KeyboardInterrupt:
            return 'panic'



if __name__ == "__main__":
    main()
