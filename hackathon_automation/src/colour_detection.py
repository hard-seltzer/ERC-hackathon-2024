#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class ColorDetector(Node):
    def __init__(self):
        super().__init__('colour_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(String, '/task_status', 10)
        self.bridge = CvBridge()
        self.cone_count = 0

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define color ranges for red and blue cones
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        lower_blue = np.array([100, 100, 100])
        upper_blue = np.array([130, 255, 255])

        # Create masks for red and blue colors
        red_mask = cv2.inRange(hsv_image, lower_red, upper_red)
        blue_mask = cv2.inRange(hsv_image, lower_blue, upper_blue)

        # Find contours in the masks
        red_contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        blue_contours, _ = cv2.findContours(blue_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        status_msg = String()

        if len(red_contours) > 0:
            status_msg.data = "Error detected at site"
            self.cone_count += 1
        elif len(blue_contours) > 0:
            status_msg.data = "No error detected"
            self.cone_count += 1
        else:
            return

        self.publisher.publish(status_msg)
        self.get_logger().info(f'Published status: {status_msg.data}')

        if self.cone_count == 5:
            final_msg = String()
            final_msg.data = "Surveillance task completed"
            self.publisher.publish(final_msg)
            self.get_logger().info('Surveillance task completed')

def main(args=None):
    rclpy.init(args=args)
    color_detector = ColorDetector()
    rclpy.spin(color_detector)
    color_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()