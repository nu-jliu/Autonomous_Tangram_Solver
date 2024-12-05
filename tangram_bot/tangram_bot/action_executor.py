import rclpy
from rclpy.node import Node
from rclpy.task import Future
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle, GoalResponse, CancelResponse

from std_srvs.srv import Trigger, Trigger_Request, Trigger_Response

from tangram_msgs.msg import RobotAction, PickPlace
from tangram_msgs.action import ExecuteAction
from rcl_interfaces.msg import ParameterDescriptor

from google_speech import Speech

from tangram_bot.robot_commander import RobotCommander

import copy
import time


def get_shape_name(index):
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
        self.in_execution = False
        self.actions_ready = False
        self.cancel_requested = False
        self.in_motion = False
        self.calibration_ready = False
        self.robot_action: RobotAction = None

        # self.z_standoff = 0.12
        # self.z_down = 0.06

        self.declare_parameter(
            "standoff",
            0.12,
            ParameterDescriptor(description="Standoff height"),
        )
        self.declare_parameter(
            "object",
            0.06,
            ParameterDescriptor(description="Object height"),
        )

        self.z_standoff = (
            self.get_parameter("standoff").get_parameter_value().double_value
        )
        self.z_down = self.get_parameter("object").get_parameter_value().double_value

        self.commander = RobotCommander()

        self.sub_robot_action = self.create_subscription(
            RobotAction,
            "robot/action",
            self.sub_robot_action_callback,
            10,
        )

        self.srv_reset = self.create_service(Trigger, "reset", self.srv_reset_callback)
        self.srv_ready = self.create_service(Trigger, "ready", self.srv_ready_callback)

        self.action_execute_action = ActionServer(
            self,
            ExecuteAction,
            "action/execute",
            goal_callback=self.action_execute_action_goal_callback,
            cancel_callback=self.action_execute_action_cancel_callback,
            execute_callback=self.action_execute_action_execute_callback,
        )

        self.cli_piece_segment_reset = self.create_client(
            Trigger,
            "piece/segment/reset",
        )

        self.cli_piece_detection_reset = self.create_client(
            Trigger,
            "piece/detection/reset",
        )

        self.cli_piece_p2r_reset = self.create_client(
            Trigger,
            "piece/p2r/reset",
        )

        self.cli_puzzle_segment_reset = self.create_client(
            Trigger,
            "puzzle/segment/reset",
        )

        self.cli_puzzle_solver_reset = self.create_client(
            Trigger,
            "puzzle/solver/reset",
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

    def sub_robot_action_callback(self, msg: RobotAction):
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
        self.calibration_ready = True

        response.success = True
        return response

    def action_execute_action_goal_callback(self, request):
        self.get_logger().info(f"Received goal request: {request}")

        if self.in_motion:
            return GoalResponse.REJECT

        else:
            self.cancel_requested = False
            self.in_motion = True

            return GoalResponse.ACCEPT

    def action_execute_action_cancel_callback(self, request):
        self.get_logger().info(f"Received cancel request: {request}")

        if self.cancel_requested:
            return CancelResponse.REJECT

        else:
            self.cancel_requested = True
            return CancelResponse.ACCEPT

    def action_execute_action_execute_callback(self, goal_handle: ServerGoalHandle):
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

        stage = 1

        for action in actions:
            action: PickPlace

            pick_x = action.pick.x
            pick_y = action.pick.y

            place_x = action.place.x
            place_y = action.place.y

            # motion = 1

            shape_name = get_shape_name(action.shape)
            speech = Speech(f"Picking {shape_name}", "en")
            speech.play()

            if self.cancel_requested:
                return self.get_cancel_result(goal_handle)
            self.commander.move_arm(pick_x, pick_y, self.z_standoff, pick=True)
            self.publish_status(f"Stage {stage} --> Pick standoff", goal_handle)
            # motion += 1

            # time.sleep(0.5)
            if self.cancel_requested:
                return self.get_cancel_result(goal_handle)
            self.commander.grasp()
            self.publish_status(f"Stage {stage} --> Grasp", goal_handle)
            # motion += 1

            if self.cancel_requested:
                return self.get_cancel_result(goal_handle)
            self.commander.move_arm(pick_x, pick_y, self.z_down, pick=True)
            self.publish_status(f"Stage {stage} --> Pick object", goal_handle)
            # motion += 1

            # time.sleep(0.5)

            if self.cancel_requested:
                return self.get_cancel_result(goal_handle)
            self.commander.move_arm(pick_x, pick_y, self.z_standoff, pick=True)
            self.publish_status(f"Stage {stage} --> Pick standoff", goal_handle)
            # motion += 1

            # time.sleep(0.5)

            # if self.cancel_requested:
            #     return self.get_cancel_result(goal_handle)
            # self.publish_status(f"Stage {stage} --> Motion {motion}", goal_handle)
            # # motion += 1
            # self.commander.go_home()

            # time.sleep(0.5)
            speech = Speech(f"Placing {shape_name}", "en")
            speech.play()

            if self.cancel_requested:
                return self.get_cancel_result(goal_handle)
            self.commander.move_arm(place_x, place_y, self.z_standoff, pick=False)
            self.publish_status(f"Stage {stage} --> Place standoff", goal_handle)
            # motion += 1

            # time.sleep(0.5)

            if self.cancel_requested:
                return self.get_cancel_result(goal_handle)
            self.commander.move_arm(place_x, place_y, self.z_down, pick=False)
            self.publish_status(f"Stage {stage} --> Place object", goal_handle)
            # motion += 1

            # time.sleep(0.5)
            if self.cancel_requested:
                return self.get_cancel_result(goal_handle)
            self.commander.release()
            self.publish_status(f"Stage {stage} --> Release", goal_handle)
            # motion += 1

            if self.cancel_requested:
                return self.get_cancel_result(goal_handle)
            self.commander.move_arm(place_x, place_y, self.z_standoff, pick=False)
            self.publish_status(f"Stage {stage} --> Place object", goal_handle)
            # motion += 1

            stage += 1

        if self.cancel_requested:
            return self.get_cancel_result(goal_handle)
        self.commander.go_home()
        self.publish_status("Final --> Going home", goal_handle)
        # motion += 1

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

        :param status: _description_
        :type status: str
        :param goal_handle: _description_
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


def main(args=None):
    rclpy.init(args=args)
    node = ActionExecutor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
