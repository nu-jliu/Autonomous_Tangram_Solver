import os
import time
import numpy as np

import torch
from sam2.build_sam import build_sam2
from sam2.automatic_mask_generator import SAM2AutomaticMaskGenerator

import rclpy
from rclpy.node import Node

import cv_bridge
import cv2

from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image


class PieceSegment(Node):

    def __init__(self):
        super().__init__("piece_segment")

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

        self.get_logger().info(f"Model file path {model_path}")
        sam2 = build_sam2(model_cfg, model_path, device=device)
        self.generator = SAM2AutomaticMaskGenerator(sam2)

        self.bridge = cv_bridge.CvBridge()
        self.cv_image = None

        self.timer = self.create_timer(3.5, self.timer_callback)

        self.sub_image_piece_scan = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self.sub_image_piece_scan_callback,
            10,
        )
        self.pub_image_piece_mask = self.create_publisher(
            Image,
            "image/piece/mask",
            10,
        )
        self.pub_image_piece_segment = self.create_publisher(
            Image,
            "image/piece/segment",
            10,
        )

    def timer_callback(self):
        if self.cv_image is not None:
            self.get_logger()
            start_time = time.time()
            masks = self.generator.generate(self.cv_image)
            end_time = time.time()

            self.get_logger().info(f"Inference takes {end_time - start_time} seconds")

            w, h, _ = self.cv_image.shape
            segmented_image = np.zeros_like(self.cv_image)
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
                masked_image = cv2.bitwise_and(self.cv_image, self.cv_image, mask=mask)
                segmented_image = cv2.addWeighted(
                    segmented_image,
                    1.0,
                    masked_image,
                    1.0,
                    0,
                )

            # img_masks = self.show_anns(masks)

            # if img_masks is None:
            #     return

            # img_masks = img_masks[:, :, :3]
            # img_masks = cv2.cvtColor(img_masks, cv2.COLOR_RGB2BGR)

            img_mask_msg = self.bridge.cv2_to_imgmsg(mask_image, "mono8")
            img_segment_msg = self.bridge.cv2_to_imgmsg(segmented_image, "bgr8")
            self.pub_image_piece_segment.publish(img_segment_msg)
            self.pub_image_piece_mask.publish(img_mask_msg)
            # mask = masks[0]

    def sub_image_piece_scan_callback(self, msg: Image):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # self.cv_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        # self.get_logger().info("Image converted")

    def show_anns(self, anns, borders=True):
        if len(anns) == 0:
            return
        sorted_anns = sorted(anns, key=(lambda x: x["area"]), reverse=True)
        # ax = plt.gca()
        # ax.set_autoscale_on(False)

        img = np.ones(
            (
                sorted_anns[0]["segmentation"].shape[0],
                sorted_anns[0]["segmentation"].shape[1],
                4,
            )
        )
        img[:, :, 3] = 0
        for ann in sorted_anns:
            m = ann["segmentation"]
            color_mask = np.concatenate([np.random.random(3), [0.5]])
            img[m] = color_mask
            if borders:
                contours, _ = cv2.findContours(
                    m.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
                )
                # Try to smooth contours
                contours = [
                    cv2.approxPolyDP(contour, epsilon=0.01, closed=True)
                    for contour in contours
                ]
                cv2.drawContours(img, contours, -1, (0, 0, 1, 0.4), thickness=1)

        return img


def main(args=None):
    rclpy.init(args=args)
    node = PieceSegment()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
    finally:
        rclpy.shutdown()
