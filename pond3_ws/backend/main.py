import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import uvicorn

from std_srvs.srv import Empty
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
import threading
import asyncio

class MoveRobotNode(Node):
    def __init__(self):
        super().__init__('move_robot_node')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.velocity_callback, 10)
        self.emergency_stop_service = self.create_service(Empty, 'emergency_stop', self.handle_emergency_stop)
        self.emergency_stop_client = self.create_client(Empty, 'emergency_stop')
        while not self.emergency_stop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for emergency_stop service...')
        self.emergency_triggered = False
        self.odometry_msg = None
        self.linear_speed = None

    def update_odometry(self, msg, linear_speed):
        self.odometry_msg = msg
        self.linear_speed = linear_speed

    def move_forward(self):
        msg = Twist()
        msg.linear.x = 0.2
        self.publisher_.publish(msg)

    def turn_left(self):
        msg = Twist()
        msg.angular.z = 0.4
        self.publisher_.publish(msg)

    def turn_right(self):
        msg = Twist()
        msg.angular.z = -0.4
        self.publisher_.publish(msg)

    def backward(self):
        msg = Twist()
        msg.linear.x = -0.2
        self.publisher_.publish(msg)

    def brake(self):
        msg = Twist()
        msg.linear.x = 0.0
        self.publisher_.publish(msg)

    def emergency(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

    def handle_emergency_stop(self, request, response):
        self.emergency()
        self.get_logger().info('Emergency stop triggered via service.')
        self.emergency_triggered = True
        return response

    def call_emergency_stop(self):
        request = Empty.Request()
        self.emergency()
        future = self.emergency_stop_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Emergency stop triggered via service.')
        else:
            self.get_logger().warning('Service call failed.')

    def velocity_callback(self, msg):
        odometry_info = ""
        if self.odometry_msg is not None:
            odometry_info = f'Dados robô: Velocidade: {self.linear_speed:.2f} m/s, Posição x={self.odometry_msg.pose.pose.position.x:.2f}, y={self.odometry_msg.pose.pose.position.y:.2f}'
        self.get_logger().info(f'Inputs do robô: Velocidade Linear.x = {msg.linear.x}, Angular.z = {msg.angular.z}')
        self.get_logger().info(odometry_info)

    def begin_bot():
        rclpy.init()
        el_bot = MoveRobotNode()
        return el_bot
    
    def keep_spinning(bot):
        rclpy.spin(bot)
        bot.destroy_node()
        rclpy.shutdown()
      

app = FastAPI()
simulated_bot = MoveRobotNode.begin_bot()


# Aqui é feito o uso de uma thread pra fazer o nó do robô ficar rodando em paralelo com o servidor web 
@app.on_event("startup")
async def startup_event():
    threading.Thread(target=rclpy.spin, args=(simulated_bot,), daemon=True).start()

# Aqui é feito a destruição do nó do robô quando o servidor web é desligado 
@app.on_event("shutdown")
def shutdown_event():
    simulated_bot.destroy_node()
    rclpy.shutdown()


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    try:
        while True:
            data = await websocket.receive_text()
            if data == "move_forward":
                simulated_bot.move_forward()
            elif data == "turn_left":
                simulated_bot.turn_left()
            elif data == "turn_right":
                simulated_bot.turn_right()
            elif data == "backward":
                simulated_bot.backward()
            elif data == "brake":
                simulated_bot.brake()
            elif data == "emergency_stop":
                simulated_bot.call_emergency_stop()
            await websocket.send_text(f"Command received: {data}")
    except WebSocketDisconnect:
        await websocket.close()

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
