#!/usr/bin/python

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from tangram_msgs.msg import Point2D

from tangram_bot.robot_commander import RobotCommander
from std_srvs.srv import Trigger

from enum import Enum, auto

import time


class States(Enum):
    Initialize = (auto(),)
    Start = (auto(),)
    Scan = (auto(),)
    Save = (auto(),)
    Idle = auto()


class ScanAprilTag(Node):

    def __init__(self):
        super().__init__("scan_apriltag")

        self.apriltag_pos = None
        self.apriltag_ready = False
        self.robot_ready = False

        self.callback_group = ReentrantCallbackGroup()

        self.commander = RobotCommander(self)
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

        while not self.cli_save.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(
                f"Service {self.cli_save.srv_name} not available, waiting again"
            )

        # asyncio.run(self.commander.go_home())
        # asyncio.run(self.commander.move_arm(0.25, -0.1, 0.1, pick=False))

        self.state = States.Initialize

    async def timer_callback(self):
        if self.state == States.Initialize:
            if not self.robot_ready:
                self.robot_ready = True

                future = self.commander.go_home()
                await future

                future = self.commander.move_arm(0.25, -0.1, 0.1, pick=False)
                await future

                self.state = States.Scan

        elif self.state == States.Start:
            self.start_time = time.time()
            self.counter = 0
            self.state = States.Scan

        elif self.state == States.Scan:
            if self.apriltag_pos is not None:
                diff = time.time() - self.start_time
                self.counter += 1

                self.apriltag_pos = None

                if diff > 10.0 and self.counter > 100:
                    self.state = States.Save

        elif self.state == States.Save:
            if not self.apriltag_ready:
                self.apriltag_ready = True

                self.get_logger().info("Saving apriltag")
                future = self.cli_ready.call_async(Trigger.Request())
                await future

                future = self.cli_save.call_async(Trigger.Request())
                await future

                await self.commander.go_home()
                self.state = States.Idle

        elif self.state == States.Idle:
            # Do nothing
            pass

        else:
            self.get_logger().error("Invalid state")

        # elif not self.apriltag_ready:
        #     if self.apriltag_pos is not None:
        #         if self.start_time == 0.0:
        #             self.start_time = time.time()

        #         diff = time.time() - self.start_time
        #         self.counter += 1
        #         # if self.counter > 20:
        #         if diff > 10.0 and self.counter > 100:
        #             await self.commander.go_home()

        #             self.get_logger().info("Saving apriltag")
        #             future = self.cli_ready.call_async(Trigger.Request())
        #             await future

        #             future = self.cli_save.call_async(Trigger.Request())
        #             await future

        #             self.apriltag_ready = True

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
