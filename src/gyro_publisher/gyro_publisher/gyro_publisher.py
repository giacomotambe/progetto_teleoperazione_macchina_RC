import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random
import time

class GyroPublisherNode(Node):
    def __init__(self):
        super().__init__('gyro_publisher')
        self.publisher_ = self.create_publisher(String, 'gyro_data', 10)
        timer_period = 0.01  # secondi
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.get_logger().info('GyroPublisherNode has been started.')

    def timer_callback(self):
        # Simula la lettura dei dati del giroscopio
        pitch = round(random.uniform(-180.0, 180.0), 2)
        roll = round(random.uniform(-180.0, 180.0), 2)
        yaw = round(random.uniform(-180.0, 180.0), 2)
        
        gyro_data = f"pitch: {pitch}, roll: {roll}, yaw: {yaw}"
        msg = String()
        msg.data = gyro_data
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    node = GyroPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
