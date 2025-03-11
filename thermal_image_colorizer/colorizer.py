import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from std_msgs.msg import String

class ColormapImagePublisher(Node):
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

            # Apply colormap
            if self.colormap_name == "Grayscale":
                # For grayscale, we don't apply a colormap, but convert to RGB
                colored_img = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
            else:
                # Map colormap name to OpenCV constant
                colormap_map = {
                    "Inferno": cv2.COLORMAP_INFERNO,
                    "Jet": cv2.COLORMAP_JET,
                    "Viridis": cv2.COLORMAP_VIRIDIS,
                    "Rainbow": cv2.COLORMAP_RAINBOW
                }
                colormap = colormap_map.get(self.colormap_name, cv2.COLORMAP_INFERNO)
                colored_img = cv2.applyColorMap(cv_image, colormap)

            # Convert the colored image back to ROS message
            colormapped_image_msg = self.bridge.cv2_to_imgmsg(colored_img, encoding='bgr8')

            # Publish the colormapped image
            self.colormapped_image_publisher.publish(colormapped_image_msg)

            # Publish the colormap name (optional)
            colormap_name_msg = String()
            colormap_name_msg.data = self.colormap_name
            self.colormap_name_publisher.publish(colormap_name_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ColormapImagePublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
