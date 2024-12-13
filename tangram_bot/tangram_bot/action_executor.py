import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.task import Future
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle, GoalResponse, CancelResponse

from std_srvs.srv import Trigger, Trigger_Request, Trigger_Response

from tangram_msgs.msg import RobotAction, PickPlace, Point2D
from tangram_msgs.action import ExecuteAction
from rcl_interfaces.msg import ParameterDescriptor

from google_speech import Speech

from tangram_bot.robot_commander import RobotCommander

import copy
import time


def get_shape_name(index):
    """Get the name for each tangram shape

    :param index: Index of the tangram piece
    :type index: int
    :return: Name of the tangram piece
    :rtype: str
    """
    shapes = [
        "Small Triangle",
        "Medium Triangle",
        "Large Triangle",
        "Square",
        "Parallelogram",
    ]

    return shapes[index] if index < 5 else shapes[0]


class ActionExecutor(Node):

    def __init__(self):
        super().__init__("action_executor")
        self.callback_group = None
        self.in_execution = False
        self.actions_ready = False
        self.cancel_requested = False
        self.in_motion = False
        self.calibration_ready = False
        self.robot_action: RobotAction = None

        self.robot_x = 0.0
        self.robot_y = 0.0

        self.declare_parameter(
            "pick.standoff",
            0.15,
            ParameterDescriptor(description="Standoff height"),
        )
        self.declare_parameter(
            "pick.object",
            0.06,
            ParameterDescriptor(description="Object height"),
        )
        self.declare_parameter(
            "place.standoff",
            0.15,
            ParameterDescriptor(description="Standoff height"),
        )
        self.declare_parameter(
            "place.object",
            0.06,
            ParameterDescriptor(description="Object height"),
        )

        self.pick_standoff = (
            self.get_parameter("pick.standoff").get_parameter_value().double_value
        )
        self.pick_object = (
            self.get_parameter("pick.object").get_parameter_value().double_value
        )
        self.place_standoff = (
            self.get_parameter("place.standoff").get_parameter_value().double_value
        )
        self.place_object = (
            self.get_parameter("place.object").get_parameter_value().double_value
        )

        self.commander = RobotCommander(self)

        self.sub_robot_pose = self.create_subscription(
            Point2D,
            "robot/pose",
            self.sub_robot_pose_callback,
            10,
            callback_group=self.callback_group,
        )
        self.sub_robot_action = self.create_subscription(
            RobotAction,
            "robot/action",
            self.sub_robot_action_callback,
            10,
            callback_group=self.callback_group,
        )

        self.srv_reset = self.create_service(
            Trigger,
            "reset",
            self.srv_reset_callback,
            callback_group=self.callback_group,
        )
        self.srv_ready = self.create_service(
            Trigger,
            "ready",
            self.srv_ready_callback,
            callback_group=self.callback_group,
        )

        self.action_execute_action = ActionServer(
            self,
            ExecuteAction,
            "action/execute",
            goal_callback=self.action_execute_action_goal_callback,
            cancel_callback=self.action_execute_action_cancel_callback,
            execute_callback=self.action_execute_action_execute_callback,
            callback_group=self.callback_group,
        )

        self.cli_piece_segment_reset = self.create_client(
            Trigger,
            "piece/segment/reset",
            callback_group=self.callback_group,
        )

        self.cli_piece_detection_reset = self.create_client(
            Trigger,
            "piece/detection/reset",
            callback_group=self.callback_group,
        )

        self.cli_piece_p2r_reset = self.create_client(
            Trigger,
            "piece/p2r/reset",
            callback_group=self.callback_group,
        )

        self.cli_puzzle_segment_reset = self.create_client(
            Trigger,
            "puzzle/segment/reset",
            callback_group=self.callback_group,
        )

        self.cli_puzzle_solver_reset = self.create_client(
            Trigger,
            "puzzle/solver/reset",
            callback_group=self.callback_group,
        )

        while not self.cli_piece_segment_reset.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                f"Service {self.cli_piece_segment_reset.srv_name} not available, waiting again"
            )

        while not self.cli_piece_detection_reset.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                f"Service {self.cli_piece_detection_reset.srv_name} not available, waiting again"
            )

        while not self.cli_piece_p2r_reset.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                f"Service {self.cli_piece_p2r_reset.srv_name} not available, waiting again"
            )

        while not self.cli_puzzle_segment_reset.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                f"Service {self.cli_puzzle_segment_reset.srv_name} not available, waiting again"
            )

        while not self.cli_puzzle_solver_reset.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                f"Service {self.cli_puzzle_solver_reset.srv_name} not available, waiting again"
            )

    def sub_robot_pose_callback(self, msg: Point2D):
        """Subcription callback of the robot pose

        :param msg: Robot pose object
        :type msg: Point2D
        """
        self.robot_x = msg.x
        self.robot_y = msg.y

    def sub_robot_action_callback(self, msg: RobotAction):
        if len(msg.actions) < 7:
            return

        self.robot_action = msg

        if not self.actions_ready:
            speech = Speech("Actions received, ready to move", "en")
            speech.play()

            self.actions_ready = True

    async def srv_reset_callback(
        self,
        request: Trigger_Request,
        response: Trigger_Response,
    ):
        """Reset the node

        :param request: Request object for reset service
        :type request: Trigger_Request
        :param response: Response object for the reset service
        :type response: Trigger_Response
        :return: Response object of the service
        :rtype: Trigger_Response
        """
        self.get_logger().info("Resetting the system")

        future: Future = None
        req = Trigger.Request()

        future = self.cli_piece_segment_reset.call_async(req)
        await future

        future = self.cli_piece_detection_reset.call_async(req)
        await future

        future = self.cli_piece_p2r_reset.call_async(req)
        await future

        future = self.cli_puzzle_segment_reset.call_async(req)
        await future

        future = self.cli_puzzle_solver_reset.call_async(req)
        await future

        response.success = True
        return response

    def srv_ready_callback(self, request: Trigger_Request, response: Trigger_Response):
        """Set the calibration to ready

        :param request: Request object of the ready service
        :type request: Trigger_Request
        :param response: Response object of the readu service
        :type response: Trigger_Response
        :return: Response object
        :rtype: Trigger_Response
        """
        self.calibration_ready = True

        response.success = True
        return response

    def action_execute_action_goal_callback(self, request):
        """Goal handle of the action server

        :param request: Goal request object of the action
        :type request: RobotAction_Goal
        :return: Goal response object
        :rtype: GoalResponse
        """
        self.get_logger().info(f"Received goal request: {request}")

        if self.in_motion:
            return GoalResponse.REJECT

        else:
            self.cancel_requested = False
            self.in_motion = True

            return GoalResponse.ACCEPT

    def action_execute_action_cancel_callback(self, request):
        """Cancel the action

        :param request: Request object of the action
        :type request: RobotAction_Goal
        :return: Cancel response object
        :rtype: CancelResponse
        """
        self.get_logger().info(f"Received cancel request: {request}")

        if self.cancel_requested:
            return CancelResponse.REJECT

        else:
            self.cancel_requested = True
            return CancelResponse.ACCEPT

    async def action_execute_action_execute_callback(
        self,
        goal_handle: ServerGoalHandle,
    ):
        """Execute the robot action

        :param goal_handle: Goal handle object of the server
        :type goal_handle: ServerGoalHandle
        :return: Success result object
        :rtype: RobotAction_Result
        """
        if self.robot_action is None:
            speech = Speech("Action not received yet, please wait", "en")
            speech.play()

            return self.get_fail_result(goal_handle)

        if not self.calibration_ready:
            speech = Speech("Not calibrated yet, please wait", "en")
            speech.play()

            return self.get_fail_result(goal_handle)

        self.get_logger().info("Starting execution")

        speech = Speech(f"Start solving puzzle", "en")
        speech.play()

        actions = copy.deepcopy(self.robot_action.actions)

        # stage = 1

        for action in actions:
            action: PickPlace

            pick_x = action.pick.x
            pick_y = action.pick.y

            place_x = action.place.x
            place_y = action.place.y

            if action.shape == PickPlace.LARGE_TRIANGLE:
                pick_x += 0.01

            shape_name = get_shape_name(action.shape)
            speech = Speech(f"Picking {shape_name}", "en")
            speech.play()

            stage = shape_name

            if self.cancel_requested:
                return self.get_cancel_result(goal_handle)

            await self.commander.move_arm(
                pick_x,
                pick_y,
                self.pick_standoff,
                pick=True,
            )
            self.publish_status(f"Stage {stage} --> Pick standoff", goal_handle)

            if self.cancel_requested:
                return self.get_cancel_result(goal_handle)
            await self.commander.grasp()
            self.publish_status(f"Stage {stage} --> Grasp", goal_handle)

            if self.cancel_requested:
                return self.get_cancel_result(goal_handle)
            await self.commander.move_arm(pick_x, pick_y, self.pick_object, pick=True)
            self.publish_status(f"Stage {stage} --> Pick object", goal_handle)

            if self.cancel_requested:
                return self.get_cancel_result(goal_handle)
            await self.commander.move_arm(
                pick_x,
                pick_y,
                self.pick_standoff,
                pick=True,
            )
            self.publish_status(f"Stage {stage} --> Pick standoff", goal_handle)

            speech = Speech(f"Placing {shape_name}", "en")
            speech.play()

            if self.cancel_requested:
                return self.get_cancel_result(goal_handle)
            await self.commander.move_arm(
                place_x, place_y, self.place_standoff, pick=False
            )
            self.publish_status(f"Stage {stage} --> Place standoff", goal_handle)

            if self.cancel_requested:
                return self.get_cancel_result(goal_handle)
            await self.commander.move_arm(
                place_x,
                place_y,
                self.place_object,
                pick=False,
            )
            self.publish_status(f"Stage {stage} --> Place object", goal_handle)

            if self.cancel_requested:
                return self.get_cancel_result(goal_handle)
            await self.commander.release()
            self.publish_status(f"Stage {stage} --> Release", goal_handle)

            if self.cancel_requested:
                return self.get_cancel_result(goal_handle)
            await self.commander.move_arm(
                place_x, place_y, self.place_standoff, pick=False
            )
            self.publish_status(f"Stage {stage} --> Place object", goal_handle)

        if self.cancel_requested:
            return self.get_cancel_result(goal_handle)
        await self.commander.go_home()
        self.publish_status("Final --> Going home", goal_handle)

        speech = Speech("Finished solving puzzle", "en")
        speech.play()

        return self.get_success_result(goal_handle)

    def get_success_result(self, goal_handle: ServerGoalHandle):
        """Contruct a success result of the action

        :param goal_handle: Goal handle object of the action
        :type goal_handle: ServerGoalHandle
        :return: Result of the action
        :rtype: ExecuteAction_Result
        """
        goal_handle.succeed()
        result = ExecuteAction.Result()
        result.success = True
        self.in_motion = False
        return result

    def get_fail_result(self, goal_handle: ServerGoalHandle):
        """Contruct a fail result of the action

        :param goal_handle: Goal handle object of the action
        :type goal_handle: ServerGoalHandle
        :return: Result of the action
        :rtype: ExecuteAction_Result
        """
        goal_handle.abort()
        result = ExecuteAction.Result()
        result.success = False
        self.in_motion = False
        return result

    def get_cancel_result(self, goal_handle: ServerGoalHandle):
        """Contruct a cancel result of the action

        :param goal_handle: Goal handle object of the action
        :type goal_handle: ServerGoalHandle
        :return: Result of the action
        :rtype: ExecuteAction_Result
        """
        self.get_logger().warn("Canceling the goal")
        goal_handle.canceled()
        result = ExecuteAction.Result()
        result.success = False
        self.in_motion = False
        self.cancel_requested = False
        return result

    def publish_status(self, status: str, goal_handle: ServerGoalHandle):
        """Publish the status of the server

        :param status: The status
        :type status: str
        :param goal_handle: Goal handle object
        :type goal_handle: ServerGoalHandle
        """
        self.status = status

        feedback = ExecuteAction.Feedback()
        feedback.status = status
        feedback.x = self.commander.x
        feedback.y = self.commander.y
        feedback.z = self.commander.z

        self.get_logger().info(f"In state: {status}")
        goal_handle.publish_feedback(feedback)

    async def move_to(self, x: float, y: float, z: float, pick: bool = False):
        """Move to a position

        :param x: X position
        :type x: float
        :param y: Y position
        :type y: float
        :param z: Z position
        :type z: float
        :param pick: Whether the moved position is pick, defaults to False
        :type pick: bool, optional
        :return: Actual x, y coordinate
        :rtype: tuple[int, int]
        """
        await self.commander.move_arm(x, y, z, pick)

        time.sleep(0.2)

        diff_x = x - self.robot_x
        diff_y = y - self.robot_y

        await self.commander.move_offset(diff_x, diff_y, 0.0, pick)

        return x + diff_x, y + diff_y


def main(args=None):
    """Main function of the node

    :param args: ROS args, defaults to None
    :type args: list[str], optional
    """
    rclpy.init(args=args)
    node = ActionExecutor()

    try:
        # rclpy.spin(node)
        rclpy.spin(node, MultiThreadedExecutor())
    except KeyboardInterrupt:
        node.destroy_node()
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
