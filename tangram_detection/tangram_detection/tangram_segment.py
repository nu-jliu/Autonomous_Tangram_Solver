#!/bin/python

import os
import torch

from network import CAE

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge

from rcl_interfaces.msg import ParameterDescriptor
from tangram_msgs.srv import SetTarget, SetTarget_Request, SetTarget_Response


class TangramSegment(Node):

    def __init__(self) -> None:
        super().__init__("tangram_segment")

        self.declare_parameter(
            "model_dir",
            "",
            ParameterDescriptor(description="Directory of the model file"),
        )
        self.declare_parameter(
            "model_filename",
            "",
            ParameterDescriptor(description="Filename of the model"),
        )
        self.declare_parameter(
            "image_dir",
            "",
            ParameterDescriptor(description="Directory of target images"),
        )

        model_dir = self.get_parameter("model_dir").get_parameter_value().string_value
        model_filename = (
            self.get_parameter("model_filename").get_parameter_value().string_value
        )
        self.image_dir = (
            self.get_parameter("image_dir").get_parameter_value().string_value
        )

        self.model_path = os.path.join(model_dir, model_filename)
        self.model = CAE(input_channels=1)
        self.model.load_state_dict(torch.load(self.model_path, weights_only=True))

        self.input_tensor = None
        self.bridge = CvBridge()

    def timer_callback(self):
        if self.input_tensor is not None:
            segment_image = self.model.forward(self.input_tensor)
            self.bridge.cv2_to_imgmsg()

    # def srv_set_target_callback(self, )
