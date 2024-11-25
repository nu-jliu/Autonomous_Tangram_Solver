import os
import time
import numpy as np

import torch
from sam2.build_sam import build_sam2
from sam2.automatic_mask_generator import SAM2AutomaticMaskGenerator

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import cv_bridge
import cv2

from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image


class PieceSegment(Node):

    def __init__(self):
        super().__init__("piece_segment")
        self.callback_group = ReentrantCallbackGroup()

        self.declare_parameter(
            "model_dir",
            "",
            ParameterDescriptor(description="Directory of the model"),
        )
        self.declare_parameter(
            "model_file",
            "",
            ParameterDescriptor(description="File name of the model file"),
        )
        self.declare_parameter(
            "model_cfg",
            "",
            ParameterDescriptor(description="Path to the configuration file"),
        )

        model_dir = self.get_parameter("model_dir").get_parameter_value().string_value
        model_file = self.get_parameter("model_file").get_parameter_value().string_value
        model_cfg = self.get_parameter("model_cfg").get_parameter_value().string_value

        model_path = os.path.join(model_dir, model_file)

        np.random.seed(3)
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        if device.type == "cuda":
            # use bfloat16 for the entire notebook
            torch.autocast("cuda", dtype=torch.bfloat16).__enter__()
            # turn on tfloat32 for Ampere GPUs (https://pytorch.org/docs/stable/notes/cuda.html#tensorfloat-32-tf32-on-ampere-devices)
            if torch.cuda.get_device_properties(0).major >= 8:
                torch.backends.cuda.matmul.allow_tf32 = True
                torch.backends.cudnn.allow_tf32 = True

        self.get_logger().info(f"Loaded checkpoint from file {model_path}")
        sam2 = build_sam2(model_cfg, model_path, device=device)
        self.generator = SAM2AutomaticMaskGenerator(sam2)

        self.bridge = cv_bridge.CvBridge()
        # self.cv_image = None
        self.segmented_image = None
        self.mask_image = None
        self.images_ready = False

        self.timer = self.create_timer(
            0.1,
            self.timer_callback,
            callback_group=self.callback_group,
        )

        self.sub_image_raw = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self.sub_image_raw_callback,
            10,
            # callback_group=self.callback_group,
        )

        self.pub_image_piece_scan = self.create_publisher(
            Image,
            "image/piece/scan",
            10,
            callback_group=self.callback_group,
        )
        self.pub_image_piece_mask = self.create_publisher(
            Image,
            "image/piece/mask",
            10,
            callback_group=self.callback_group,
        )
        self.pub_image_piece_segment = self.create_publisher(
            Image,
            "image/piece/segment",
            10,
            callback_group=self.callback_group,
        )

    def timer_callback(self):
        if self.segmented_image is not None and self.mask_image is not None:
            # self.get_logger()

            # img_masks = self.show_anns(masks)

            # if img_masks is None:
            #     return

            # img_masks = img_masks[:, :, :3]
            # img_masks = cv2.cvtColor(img_masks, cv2.COLOR_RGB2BGR)

            img_mask_msg = self.bridge.cv2_to_imgmsg(self.mask_image, "mono8")
            img_segment_msg = self.bridge.cv2_to_imgmsg(self.segmented_image, "bgr8")

            self.pub_image_piece_segment.publish(img_segment_msg)
            self.get_logger().info(
                f"Segment image published on {self.pub_image_piece_segment.topic_name}"
            )
            time.sleep(1.0)

            self.pub_image_piece_mask.publish(img_mask_msg)
            self.get_logger().info(
                f"Mask image published on {self.pub_image_piece_mask.topic_name}"
            )
            # mask = masks[0]

    def sub_image_raw_callback(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        cv_image = cv_image[180:540, 480:800, :]
        w, h, _ = cv_image.shape

        img_scan_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.pub_image_piece_scan.publish(img_scan_msg)

        self.get_logger().info("Starting generating masks")

        start_time = time.time()
        masks = self.generator.generate(cv_image)
        end_time = time.time()

        self.get_logger().info("Mask generation finished")

        self.get_logger().info(f"Inference takes {end_time - start_time} seconds")

        segmented_image = np.zeros_like(cv_image)
        mask_image = np.zeros((w, h), dtype=np.uint8)

        for mask_data in masks:
            if mask_data["area"] > w * h / 5:
                continue

            mask = mask_data["segmentation"]
            shape = mask.shape
            max_val = np.max(mask)
            # self.get_logger().info(f"Mask shape: {shape}, max: {max_val}")

            mask = (mask * 255).astype(np.uint8)
            mask_image = cv2.bitwise_or(mask_image, mask)
            masked_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)
            segmented_image = cv2.addWeighted(
                segmented_image,
                1.0,
                masked_image,
                1.0,
                0,
            )

        self.mask_image = mask_image
        self.segmented_image = segmented_image

        if not self.images_ready:
            self.images_ready = True
        # self.cv_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # self.get_logger().info("Image converted")


def main(args=None):
    rclpy.init(args=args)
    node = PieceSegment()

    try:
        rclpy.spin(node)
        # rclpy.spin(node, MultiThreadedExecutor())
    except KeyboardInterrupt:
        node.destroy_node()
    finally:
        rclpy.try_shutdown()
