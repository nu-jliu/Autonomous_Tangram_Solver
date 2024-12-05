#!/usr/bin/python

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from tangram_msgs.msg import Point2D

from tangram_bot.robot_commander import RobotCommander
from std_srvs.srv import Trigger

import enum
from enum import auto

import time


class ScanAprilTag(Node):

    def __init__(self):
        super().__init__("scan_apriltag")

        self.apriltag_pos = None
        self.apriltag_ready = False

        self.callback_group = ReentrantCallbackGroup()

        self.commander = RobotCommander()
        self.counter = 0
        self.start_time = 0.0

        self.timer = self.create_timer(
            0.01,
            self.timer_callback,
            # callback_group=self.callback_group,
        )

        self.apriltag_detect = self.create_subscription(
            Point2D,
            "april/detect",
            self.sub_cam_to_arm_callback,
            10,
            callback_group=self.callback_group,
        )

        self.cli_ready = self.create_client(
            Trigger,
            "ready",
            callback_group=self.callback_group,
        )

        self.cli_save = self.create_client(
            Trigger,
            "april/save",
            callback_group=self.callback_group,
        )

        while not self.cli_ready.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(
                f"Service {self.cli_ready.srv_name} not available, waiting again"
            )

        self.commander.go_home()
        self.commander.move_arm(0.2, -0.1, 0.1, pick=False)

    async def timer_callback(self):

        if not self.apriltag_ready:
            if self.apriltag_pos is not None:
                if self.start_time == 0.0:
                    self.start_time = time.time()

                diff = time.time() - self.start_time
                self.counter += 1
                # if self.counter > 20:
                if diff > 10.0 and self.counter > 100:
                    self.commander.go_home()

                    self.get_logger().info("Saving apriltag")
                    future = self.cli_ready.call_async(Trigger.Request())
                    await future

                    future = self.cli_save.call_async(Trigger.Request())
                    await future

                    self.apriltag_ready = True

    def sub_cam_to_arm_callback(self, msg: Point2D):
        self.apriltag_pos = msg


def main(args=None):
    rclpy.init(args=args)
    node = ScanAprilTag()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
