#!/usr/bin/python

import os
import numpy as np

import gc
import torch
from torchvision import transforms

from image_segmentation.network import CAE

from PIL import Image as PIL_Image

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge

from tangram_msgs.srv import SetTarget, SetTarget_Request, SetTarget_Response
from std_srvs.srv import Trigger, Trigger_Request, Trigger_Response

from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image
from tangram_msgs.msg import PuzzleShape


from google_speech import Speech


class PuzzleSegment(Node):

    def __init__(self) -> None:
        gc.collect()
        torch.cuda.empty_cache()

        super().__init__("puzzle_segment")

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

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model_path = os.path.join(model_dir, model_filename)
        self.get_logger().info(f"Loading model from {self.model_path}")
        self.model = CAE(input_channels=1)
        self.model.load_state_dict(torch.load(self.model_path, weights_only=True))
        self.model.to(self.device)
        self.get_logger().info(f"Finished looding model")

        self.current_shape = ""
        self.input_tensor = None
        self.origin_array = None
        self.input_ready = False
        # self.image_msg: Image = None
        self.input_ready = False
        self.bridge = CvBridge()

        self.transform = transforms.Compose(
            [
                transforms.Resize((256, 256)),  # Adjust size
                transforms.ToTensor(),  # Convert to tensor
            ]
        )

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.sub_puzzle_shape = self.create_subscription(
            PuzzleShape,
            "puzzle/shape",
            self.sub_puzzle_shape_callback,
            10,
        )

        self.srv_set_target = self.create_service(
            SetTarget,
            "set_target",
            self.srv_set_target_callback,
        )
        self.srv_reset = self.create_service(
            Trigger,
            "puzzle/segment/reset",
            self.srv_reset_callback,
        )

        self.pub_image_target = self.create_publisher(
            Image,
            "puzzle/image/target",
            10,
        )
        self.pub_image_inferenced = self.create_publisher(
            Image,
            "puzzle/image/inferenced",
            10,
        )

        speech = Speech("Model loaded, please select input", "en")
        speech.play()

    def timer_callback(self):
        """Timer callback of the node"""

        if self.input_tensor is not None and self.input_ready:

            segment_tensor = self.model.forward(self.input_tensor)
            segment_tensor = segment_tensor.squeeze(0).cpu()
            segment_array = (segment_tensor > 0.5).float().numpy()
            segment_array = (segment_array * 255).astype(np.uint8)
            # self.get_logger().info(f"{segment_array.shape}")

            if segment_array.ndim > 2:
                segment_array = segment_array[0]

            segment_image_msg = self.bridge.cv2_to_imgmsg(
                segment_array,
                encoding="mono8",
            )
            self.pub_image_inferenced.publish(segment_image_msg)

            origin_image_msg = self.bridge.cv2_to_imgmsg(
                self.origin_array,
                encoding="mono8",
            )
            self.pub_image_target.publish(origin_image_msg)

    def sub_puzzle_shape_callback(self, msg: PuzzleShape):
        """Subscription callback of the puzzle shape

        :param msg: subcribed message
        :type msg: PuzzleShape
        """
        shape = msg.shape
        target_path = os.path.join(self.image_dir, f"{shape}.png")

        if shape != self.current_shape:
            try:
                image = PIL_Image.open(target_path).convert("L")
                tensor = self.transform(image).unsqueeze(0)
                tensor = (tensor > 0.5).float()

                self.origin_array = tensor.squeeze(0).cpu().numpy()
                self.origin_array = (self.origin_array * 255).astype(np.uint8)

                if self.origin_array.ndim > 2:
                    self.origin_array = self.origin_array[0]

                self.input_tensor = tensor.to(self.device)
                self.input_ready = True
                self.current_shape = shape

                self.get_logger().info(f"Target shape set to {shape} successfully")
            except FileNotFoundError:
                self.get_logger().error(f"File for target {shape} not found")

    def srv_reset_callback(self, request: Trigger_Request, response: Trigger_Response):
        """Service callback of the reset service

        :param request: requet object of the service
        :type request: Trigger_Request
        :param response: response object of the service
        :type response: Trigger_Response
        :return: response object
        :rtype: Trigger_Response
        """
        self.get_logger().info("Node resetted")

        self.input_tensor = None
        self.input_ready = False

        response.success = True
        return response

    def srv_set_target_callback(
        self,
        request: SetTarget_Request,
        response: SetTarget_Response,
    ):
        """Service callback of the set target service

        :param request: Request object of the service
        :type request: SetTarget_Request
        :param response: Response object of the service
        :type response: SetTarget_Response
        :return: Response object
        :rtype: SetTarget_Response
        """
        target_name = request.target
        target_path = os.path.join(self.image_dir, f"{target_name}.png")

        try:
            image = PIL_Image.open(target_path).convert("L")
            tensor = self.transform(image).unsqueeze(0)
            tensor = (tensor > 0.5).float()

            speech = Speech(f"Received input: {target_name}", "en")
            speech.play()

            self.origin_array = tensor.squeeze(0).cpu().numpy()
            self.origin_array = (self.origin_array * 255).astype(np.uint8)

            if self.origin_array.ndim > 2:
                self.origin_array = self.origin_array[0]

            self.input_tensor = tensor.to(self.device)
            self.input_ready = True

            self.get_logger().info(f"Target set to {target_name} successfully")

            response.success = True
            return response
        except FileNotFoundError:
            self.get_logger().error(f"File for target {target_name} not found")

            response.success = False
            return response


def main(args=None):
    """Main function of the node

    :param args: arguments to the node, defaults to None
    :type args: list[str], optional
    """
    rclpy.init(args=args)
    node = PuzzleSegment()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # rclpy.node.get_logger("tangram_segment").info("Shutting down node")
        node.destroy_node()
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
