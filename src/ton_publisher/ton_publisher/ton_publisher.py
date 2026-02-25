import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial


class SerialPublisher(Node):
    def __init__(self):
        super().__init__('ton_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'channels_ton', 10)
        self.serial_port = serial.Serial('/dev/ttyACM0', 2000000, timeout=1)
        self.timer = self.create_timer(0.017, self.timer_callback)
        self.last_valid_values = [1500.0, 1500.0, 1500.0, 1500.0]  # Initialize with default values

    def read_serial_data(self):
        data_to_process = []
        try:
            # Read all available lines in the serial buffer
            while self.serial_port.in_waiting > 0:
                line = self.serial_port.readline().decode('utf-8').strip()
                data_to_process.append(line)

            # Process only the last line in the buffer
            if data_to_process:
                last_line = data_to_process[-1]
                values = list(map(float, last_line.split()))  # Convert last line to a list of floats

                # Check if values contain exactly 4 floats and are within the valid range
                if len(values) == 4 and all(v <= 2100 and v>=900 for v in values):
                    self.last_valid_values = values  # Update only if data is valid
                    self.get_logger().info(f'Received and updated values: {values}')
                else:
                    self.get_logger().warning('Invalid data received or wrong data length')

        except serial.SerialException as e:
            self.get_logger().error(f'Error reading from serial port: {e}')
        except ValueError as e:
            self.get_logger().error(f'Error processing data: {e}')

    def timer_callback(self):
        # Read data from the serial port
        self.read_serial_data()
        # Publish the last valid value
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
