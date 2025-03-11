import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import String

class ImageFlipper(Node):
    def __init__(self):
        super().__init__('colormap_image_publisher')
        self.bridge = CvBridge()

        # Declare parameters with default values
        self.declare_parameter('input_topic', '/grayscale_image')  # Topic to subscribe to
        self.declare_parameter('output_topic', '/colormapped_image')  # Topic to publish to
        self.declare_parameter('colormap_name', 'Inferno')  # Default colormap

        # Get parameters
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.colormap_name = self.get_parameter('colormap_name').get_parameter_value().string_value

        # Subscriber to grayscale 8-bit image topic
        self.image_subscriber = self.create_subscription(
            Image,
            self.input_topic,  # Get input topic from parameter
            self.image_callback,
            10
        )

        # Publisher for the colormapped image
        self.colormapped_image_publisher = self.create_publisher(
            Image,
            self.output_topic,  # Get output topic from parameter
            10
        )

        # Publisher for colormap name
        self.colormap_name_publisher = self.create_publisher(
            String,
            '/colormap_name',  # Topic to publish colormap name
            10
        )

    def image_callback(self, msg):
        try:
            # Convert the ROS image message to a CV2 image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            cv_image = cv2.rotate(cv_image, cv2.ROTATE_90_COUNTERCLOCKWISE)

            # Convert the colored image back to ROS message
            image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='mono8')

            # Publish the colormapped image
            self.colormapped_image_publisher.publish(image_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ImageFlipper()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
