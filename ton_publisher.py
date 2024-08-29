import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial


class SerialPublisher(Node):
    def __init__(self):
        super().__init__('serial_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'multifloat', 10)
        self.serial_port = serial.Serial('/dev/ttyACM0', 200000, timeout=1)
        self.timer = self.create_timer(0.017, self.timer_callback)
        self.last_valid_values = [0.0, 0.0, 0.0, 0.0]  # Inizializza con valori di default

    def read_serial_data(self):
        try:
            if self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8').strip()
                values = list(map(float, line.split()))
                if len(values) == 4 and all(v <= 2500 for v in values):
                    self.last_valid_values = values  # Aggiorna solo se i dati sono validi
                    self.get_logger().info(f'Received and updated values: {values}')
                else:
                    self.get_logger().warning('Invalid data received')
        except (ValueError, serial.SerialException) as e:
            self.get_logger().error(f'Error processing data: {e}')

    def timer_callback(self):
        # Leggi i dati dalla porta seriale
        self.read_serial_data()
        # Pubblica l'ultimo valore valido
        msg = Float32MultiArray(data=self.last_valid_values)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {self.last_valid_values}')

def main(args=None):
    rclpy.init(args=args)
    node = SerialPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
