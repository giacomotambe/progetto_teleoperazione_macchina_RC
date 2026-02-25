import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class ChannelListener(Node):

    def __init__(self):
        super().__init__('ton_listener')
        # Iscriviti al topic "/channels_ton"
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'channels_ton',
            self.listener_callback,
            10  # Dimensione della coda
        )
        self.subscription  # Prevenire il garbage collection

    def listener_callback(self, msg):
        # Verifica che il messaggio contenga esattamente 4 valori
        if len(msg.data) == 4:
            self.get_logger().info(f'Ricevuti valori: {msg.data}')
        else:
            self.get_logger().warn('Il messaggio ricevuto non contiene esattamente 4 valori!')

def main(args=None):
    rclpy.init(args=args)
    channel_listener = ChannelListener()
    
    try:
        rclpy.spin(channel_listener)
    except KeyboardInterrupt:
        pass
    finally:
        channel_listener.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
