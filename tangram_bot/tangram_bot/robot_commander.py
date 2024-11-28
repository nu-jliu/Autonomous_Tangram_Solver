import rclpy
from rclpy.task import Future
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from tangram_msgs.action import MoveArm, GoHome
from std_srvs.srv import Trigger


class RobotCommander:

    def __init__(self):
        if not rclpy.ok():
            rclpy.init()

        self.node = rclpy.create_node("robot_commander")
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

    def __del__(self):
        self.node.destroy_node()
        rclpy.try_shutdown()

    def move_arm(self, x: float, y: float, z: float, pick: bool = False):
        future: Future = None
        goal = MoveArm.Goal()
        goal.goal.x = x
        goal.goal.y = y
        goal.goal.z = z
        goal.pick = pick

        future = self.cli_action_move_arm.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)

        future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, future)

        return future.result()

    def go_home(self):
        future: Future = None
        goal = GoHome.Goal()

        future = self.cli_action_go_home.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)

        future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self.node, future)

        return future.result()

    def grasp(self):
        future: Future = None
        request = Trigger.Request()

        future = self.cli_grasp.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        return future.result()

    def release(self):
        future: Future = None
        request = Trigger.Request()

        future = self.cli_release.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)

        return future.result()
