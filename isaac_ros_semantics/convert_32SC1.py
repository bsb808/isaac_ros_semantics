#!/usr/bin/env python3
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

UPDATE_MIN, UPDATE_MAX = False, False

class ImageRepublisher(Node):
    """
    ImageRepublisher Node

    This ROS2 node processes `sensor_msgs/Image` messages where the image encoding is 
    32-bit floating point (`32FC1`) and converts them into an 8-bit grayscale image (`8UC1`).
    It performs normalization based on the minimum and maximum pixel intensity values, 
    either provided as parameters or dynamically updated from the incoming image data. 
    The processed image is then republished to a specified output topic.

    ### Parameters:
    - `input_topic` (string, default: `/semantic`): 
    The topic from which the node subscribes to `sensor_msgs/Image` messages.
    - `output_topic` (string, default: `/semantic_republished`): 
    The topic to which the processed images are published.
    - `min_value` (float, default: `None`): 
    The minimum intensity value used for normalization. If not specified, it's updated dynamically.
    - `max_value` (float, default: `None`): 
    The maximum intensity value used for normalization. If not specified, it's updated dynamically.

    ### Example Usage:
    1. Run the node with default parameters:
    ```bash
    $ ros2 run isaac_ros_semantics convert_32SC1_to_8UC1
    ```
    2. Run the node with custom parameters:
    ```bash
    $ ros2 run isaac_ros_semantics convert_32SC1_to_8UC1 --ros-args -p input_topic:=/semantic -p output_topic:=/semantic_republished
    ```
    3. View the published images:
    ```bash
    $ ros2 run image_tools showimage image:=/semantic_republished
    ```

3. In a ROS2 launch file:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='<package_name>',
            executable='convert_32sc1_to_8uc1',
            name='image_republisher',
            parameters=[
                {"input_topic": "/camera/image_raw"},
                {"output_topic": "/camera/image_processed"},
                {"min_value": 10.0},
                {"max_value": 200.0}
            ]
        )
    ])
    ```
    """
    def __init__(self):
        super().__init__("convert_32SC1_to_8UC1")

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Get configuration from ROS parameters
        self.input_topic = self.declare_parameter("input_topic", "/semantic").get_parameter_value().string_value
        self.output_topic = self.declare_parameter("output_topic", "/semantic_republished").get_parameter_value().string_value
        self.min_value = self.declare_parameter("min_value", 0.0).get_parameter_value().double_value
        self.max_value = self.declare_parameter("max_value", 10.0).get_parameter_value().double_value
        # TODO: Add a parameter to select the output encoding type - currently set to mono8.

        # Initialize subscriber and publisher
        self.subscriber = self.create_subscription(
            Image,
            self.input_topic,
            self.republish_image,
            10
        )
        self.publisher = self.create_publisher(
            Image,
            self.output_topic,
            10
        )

        self.get_logger().info(f"Subscribed to {self.input_topic}")
        self.get_logger().info(f"Publishing to {self.output_topic}")

    def republish_image(self, image_msg):

        cv_image = self.bridge.imgmsg_to_cv2(image_msg, '32SC1')
        np_image = np.array(cv_image, dtype=np.float32)
        array = np_image
        
        self.get_logger().debug(f"Min v: {np.min(array)}, Max v: {np.max(array)}")
        self.get_logger().debug(f"Min: {self.min_value}, Max: {self.max_value}")

        array = np.clip(
            (array - self.min_value) / (self.max_value - self.min_value + sys.float_info.min) * 255,
            0,
            255
        )
        array = array/10.0*255

        encoding = "mono8"
        new_msg = self.bridge.cv2_to_imgmsg(array.astype(np.uint8), encoding)
        new_msg.header = image_msg.header
        new_msg.encoding = encoding
 
        self.publisher.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImageRepublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
