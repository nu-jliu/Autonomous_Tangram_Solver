import os
import copy

import cv2

from ultralytics import YOLO
from ultralytics.engine.results import Results

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge

from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image
from tangram_msgs.msg import PuzzleShape


class ShapeDetect(Node):

    def __init__(self):
        super().__init__("shape_detect")

        self.declare_parameter(
            "model_dir",
            "",
            ParameterDescriptor(description="Directory of the model file"),
        )
        self.declare_parameter(
            "model_name",
            "shapes.pt",
            ParameterDescriptor(description="File name of the model file"),
        )

        model_dir = self.get_parameter("model_dir").get_parameter_value().string_value
        model_name = self.get_parameter("model_name").get_parameter_value().string_value
        model_path = os.path.join(model_dir, model_name)

        self.model = YOLO(model_path, task="classify", verbose=True)

        self.cv_image = None
        self.color_image = None
        self.image_ready = False

        self.bridge = CvBridge()

        self.timer = self.create_timer(0.5, self.timer_callback)

        self.sub_image_raw = self.create_subscription(
            Image,
            "/puzzle/camera/color/image_raw",
            self.sub_image_raw_callback,
            10,
        )

        self.pub_puzzle_shape = self.create_publisher(PuzzleShape, "puzzle/shape", 10)
        self.pub_image_detect = self.create_publisher(
            Image,
            "puzzle/image/detect",
            10,
        )

    def timer_callback(self):
        if self.cv_image is not None:

            cv_image = copy.deepcopy(self.cv_image)
            results: list[Results] = self.model.predict(
                source=cv_image,
                # stream=True,
                # classes=[0],
                verbose=True,
            )

            for result in results:
                names = result.cpu().names
                probs = result.cpu().probs
                boxes = result.cpu().boxes
                # top1 = probs.top1
                # top1conf = probs.top1conf
                # self.get_logger().info(f"{names}")
                # self.get_logger().info(f"{probs}")
                # self.get_logger().info(f"{top1}")
                # self.get_logger().info(f"{top1conf.item()}")
                # top1 = result.cpu().probs.top1
                # conf = result.cpu().probs.top1conf.item()
                # name = names[top1]

                for box in boxes:
                    cls = box.cls.numpy()[0]
                    name = names[cls]
                    conf = box.conf.numpy()[0]

                    if conf > 0.85:
                        self.get_logger().info(f"{name} --> {conf}")

                        msg = PuzzleShape()
                        msg.shape = name
                        msg.conf = float(conf)

                        self.pub_puzzle_shape.publish(msg)

                classified_image = result.plot()

                # if conf > 0.85:
                #     h, w, _ = classified_image.shape
                #     x = int(h / 2)
                #     y = int(w / 2)
                #     classified_image = cv2.putText(
                #         classified_image.copy(),
                #         name,
                #         (x, y),
                #         cv2.FONT_HERSHEY_COMPLEX,
                #         1,
                #         (0, 0, 255),
                #         1,
                #         cv2.LINE_AA,
                #     )

                #     msg = PuzzleShape()
                #     msg.shape = name
                #     msg.conf = conf

                #     self.pub_puzzle_shape.publish(msg)

                msg_classied = self.bridge.cv2_to_imgmsg(classified_image, "bgr8")
                self.pub_image_detect.publish(msg_classied)

    def sub_image_raw_callback(self, msg: Image):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.color_image = cv_image
        self.cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2BGRA)

        if not self.image_ready:
            self.image_ready = True


def main(args=None):
    rclpy.init(args=args)
    node = ShapeDetect()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
