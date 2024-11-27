import rclpy
from rclpy.node import Node

from tangram_msgs.msg import RobotAction, TangramPoses, TangramPose, PickPlace

import copy


def validate_poses(poses: list[TangramPose]):
    num_lt = 0
    num_mt = 0
    num_st = 0
    num_sq = 0
    num_pl = 0

    for pose in poses:
        type = pose.type

        if type == TangramPose.LARGE_TRIANGLE:
            num_lt += 1

        elif type == TangramPose.MEDIUM_TRIANGLE:
            num_mt += 1

        elif type == TangramPose.SMALL_TRIANGLE:
            num_st += 1

        elif type == TangramPose.SQUARE:
            num_sq += 1

        elif type == TangramPose.PARALELLOGRAM:
            num_pl += 1

    if num_lt != 2:
        return False

    elif num_mt != 1:
        return False

    elif num_st != 2:
        return False

    elif num_sq != 1:
        return False

    elif num_pl != 1:
        return False

    return True


class ActionGenerator(Node):

    def __init__(self):
        super().__init__("action_generator")

        self.pick_poses: TangramPoses = None
        self.place_poses: TangramPoses = None

        self.timer = self.create_timer(0.01, self.timer_callback)

        self.sub_pick_poses = self.create_subscription(
            TangramPoses,
            "pick/robot",
            self.sub_pick_poses_callback,
            10,
        )

        self.sub_place_poses = self.create_subscription(
            TangramPoses,
            "place/robot",
            self.sub_place_poses_callback,
            10,
        )

        self.pub_action = self.create_publisher(RobotAction, "robot/action", 10)

    def timer_callback(self):
        if self.pick_poses is not None and self.place_poses is not None:
            if not validate_poses(self.pick_poses.poses):
                self.get_logger().warn("Pick poses invalid")
                return

            elif not validate_poses(self.place_poses.poses):
                self.get_logger().warn("Place poses invalid")
                return

            else:
                pick_poses = copy.deepcopy(self.pick_poses.poses)
                place_poses = copy.deepcopy(self.place_poses.poses)

                actions = [
                    (pick, place)
                    for pick in pick_poses
                    for place in place_poses
                    if pick.type == place.type
                ]

                action_msg = RobotAction()
                action_msg.header.stamp = self.get_clock().now().to_msg()

                for action in actions:
                    pick, place = action

                    pick_place = PickPlace()
                    pick_place.pick.x = pick.location.x
                    pick_place.pick.y = pick.location.y

                    pick_place.place.x = place.location.x
                    pick_place.place.y = place.location.y

                    action_msg.actions.append(pick_place)

                self.pub_action.publish(action_msg)

    def sub_pick_poses_callback(self, msg: TangramPoses):
        self.pick_poses = msg

    def sub_place_poses_callback(self, msg: TangramPoses):
        self.place_poses = msg


def main(args=None):
    rclpy.init(args=args)
    node = ActionGenerator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
