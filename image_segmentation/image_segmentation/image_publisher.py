import os
import cv2

import rclpy
from rclpy.node import Node

import cv_bridge

from rcl_interfaces.msg import ParameterDescriptor


class ImagePublisher(Node):

    def __init__(self):
        super().__init__("image_publisher")

        self.declare_parameter(
            "image_dir",
            "",
            ParameterDescriptor(description="Directory containing the source image"),
        )

        self.declare_parameter(
            "image_name",
            "",
            ParameterDescriptor(descriptio="File name of the image file"),
        )

        image_dir = self.get_parameter("image_dir").get_parameter_value().string_value
        image_name = self.get_parameter("image_name").get_parameter_value().string_value

        self.image_path = os.path.join(image_dir, image_name)
