import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from tangram_msgs.action import MoveArm, GoHome
from std_srvs.srv import Trigger


class RobotCommander:

    def __init__(self, node: Node):
        if not rclpy.ok():
            rclpy.init()

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.node = node
        self.callback_group = ReentrantCallbackGroup()

        self.cli_action_move_arm = ActionClient(
            self.node,
            MoveArm,
            "move_arm",
            callback_group=self.callback_group,
        )

        self.cli_action_go_home = ActionClient(
            self.node,
            GoHome,
            "go_home",
            callback_group=self.callback_group,
        )

        self.cli_grasp = self.node.create_client(
            Trigger,
            "gripper/grasp",
            callback_group=self.callback_group,
        )

        self.cli_release = self.node.create_client(
            Trigger,
            "gripper/release",
            callback_group=self.callback_group,
        )

        while not self.cli_action_move_arm.wait_for_server(timeout_sec=10.0):
            self.node.get_logger().warn(
                f"Action server {self.cli_action_move_arm._action_name} not available, waiting again"
            )

        while not self.cli_action_go_home.wait_for_server(timeout_sec=10.0):
            self.node.get_logger().warn(
                f"Action server {self.cli_action_go_home._action_name} not available, waiting again"
            )

    async def move_arm(self, x: float, y: float, z: float, pick: bool = False):
        """Move the arm to X, Y, Z

        :param x: X position
        :type x: float
        :param y: Y position
        :type y: float
        :param z: Z position
        :type z: float
        :param pick: Whether to pick, defaults to False
        :type pick: bool, optional
        :return: Result object
        :rtype: MoveArm_Result
        """
        self.node.get_logger().info(
            f"Sending request for robot arm to move to {x, y, z}"
        )

        future: Future = None
        goal = MoveArm.Goal()
        goal.goal.x = x
        goal.goal.y = y
        goal.goal.z = z
        goal.pick = pick

        self.x = x
        self.y = y
        self.z = z

        future = self.cli_action_move_arm.send_goal_async(goal)
        await future

        future = future.result().get_result_async()
        await future

        return future.result()

    async def move_offset(
        self,
        x_diff: float,
        y_diff: float,
        z_diff: float,
        pick: bool = False,
    ):
        """Move by a offset

        :param x_diff: X difference
        :type x_diff: float
        :param y_diff: Y difference
        :type y_diff: float
        :param z_diff: Z difference
        :type z_diff: float
        :param pick: Whether to pick, defaults to False
        :type pick: bool, optional
        :return: Result object
        :rtype: MoveArm_Result
        """
        goal_x = self.x + x_diff
        goal_y = self.y + y_diff
        goal_z = self.z + z_diff

        result = await self.move_arm(goal_x, goal_y, goal_z, pick)
        return result

    async def go_home(self):
        """Go home

        :return: Result object
        :rtype: GoHome_Result
        """
        self.node.get_logger().info("Sending request to home robot arm")

        future: Future = None
        goal = GoHome.Goal()

        future = self.cli_action_go_home.send_goal_async(goal)
        await future

        future = future.result().get_result_async()
        await future

        return future.result()

    async def grasp(self):
        """Grasp an object

        :return: response object
        :rtype: Trigger_Response
        """
        self.node.get_logger().info(f"Sending request to grasp")

        future: Future = None
        request = Trigger.Request()

        future = self.cli_grasp.call_async(request)
        await future

        return future.result()

    async def release(self):
        """Release an object

        :return: Result object
        :rtype: Trigger_Response
        """
        self.node.get_logger().info("Sending request to release")

        future: Future = None
        request = Trigger.Request()

        future = self.cli_release.call_async(request)
        await future

        return future.result()
