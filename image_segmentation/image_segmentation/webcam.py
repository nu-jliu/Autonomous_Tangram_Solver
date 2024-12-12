import os
import subprocess

import cv2

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge

from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image


class WebCamera(Node):
    def __init__(self):
        super().__init__("web_camera")

        self.declare_parameter(
            "camera_name",
            "Amcrest AWC2198 USB Webcam",
            ParameterDescriptor(description="Name of the camera device"),
        )

        self.camera_name = (
            self.get_parameter("camera_name").get_parameter_value().string_value
        )

        devices = [
            f"/dev/{device}"
            for device in os.listdir("/dev")
            if device.startswith("video")
        ]

        # print(devices)
        device_info = []

        for device in devices:
            try:
                # Run v4l2-ctl to get device name
                output = subprocess.check_output(
                    ["v4l2-ctl", "--device", device, "--all"], text=True
                )
                # print(device)
                # print(output)
                for line in output.splitlines():
                    if "Card type" in line:
                        name = line.split(":", 1)[1].strip()
                        device_info.append((device, name))
                        break
            except subprocess.CalledProcessError as e:
                self.get_logger().error(f"Failed to get info for {device}: {e}")
            except FileNotFoundError:
                self.get_logger().error("v4l2-ctl not found. Please install v4l-utils.")
                self.destroy_node()
                rclpy.try_shutdown()
                return

        device_numbers = []

        # self.get_logger().info(f"{self.camera_name}")
        for device, name in device_info:
            self.get_logger().info(f"{self.camera_name, name, device}")
            if self.camera_name in name:
                # print(device)
                num_device = int(device.split("video")[1])
                device_numbers.append(num_device)

        self.device = f"/dev/video{min(device_numbers)}"

        self.cap = cv2.VideoCapture()
        self.cap.open(self.device)

        if self.cap.isOpened():
            self.get_logger().info(f"Opened device {self.device}")

        else:
            self.destroy_node()
            rclpy.try_shutdown()
            return

        self.bridge = CvBridge()

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.pub_webcam_image = self.create_publisher(
            Image,
            "puzzle/camera/color/image_raw",
            10,
        )

        # return device_info

    def timer_callback(self):
        """Timer callback"""
        try:
            ret, image = self.cap.read()

            if ret:
                image_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
                self.pub_webcam_image.publish(image_msg)

        except Exception as e:
            self.get_logger().error(f"Error: {e}")


def main(args=None):
    """Main function of the node

    :param args: ROS arguments, defaults to None
    :type args: list[str], optional
    """
    rclpy.init(args=args)
    node = WebCamera()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
