import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class VideoStreamer(Node):
    def __init__(self):
        super().__init__('video_streamer')
        
        self.subscription = self.create_subscription(
            String,
            'gyro_data',
            self.listener_callback,
            10
        )
        self.gyro_data = "No data"

        Gst.init(None)

        self.pipeline = Gst.parse_launch(
            'v4l2src device=/dev/video4 ! videoconvert ! textoverlay name=overlay valignment=top halignment=left font-desc="Sans, 24" ! x264enc tune=zerolatency ! rtph264pay ! udpsink host=172.20.10.4 port=5000'
        )

        self.textoverlay = self.pipeline.get_by_name('overlay')
        if not self.textoverlay:
            self.get_logger().error('Failed to get textoverlay element from the pipeline')
        else:
            self.get_logger().info('Successfully retrieved textoverlay element')

        self.pipeline.set_state(Gst.State.PLAYING)

    def update_overlay_text(self):
        if self.textoverlay:
            self.textoverlay.set_property('text', self.gyro_data)
            self.get_logger().info(f'Updated overlay text to: {self.gyro_data}')
        else:
            self.get_logger().error('Textoverlay element not available')

    def listener_callback(self, msg):
            self.gyro_data = msg.data
            self.get_logger().info(f'Received gyro data: {self.gyro_data}')
            self.update_overlay_text()

def main(args=None):

    rclpy.init(args=args)
    node = VideoStreamer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
