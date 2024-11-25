#ifndef MAXARM_CONTROL__ARM_CONTROL_HPP_
#define MAXARM_CONTROL__ARM_CONTROL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <CppLinuxSerial/SerialPort.hpp>
// #include "tangram_msgs/srv/move_arm.hpp"
#include "tangram_msgs/action/move_arm.hpp"
#include "tangram_msgs/srv/read_position.hpp"

namespace maxarm_control
{
class ArmControl : public rclcpp::Node
{
public:
  ArmControl();

private:
  rclcpp::Rate::SharedPtr loop_rate_;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Service<tangram_msgs::srv::ReadPosition>::SharedPtr srv_read_position_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_arm_joint_states_;

  rclcpp_action::Server<tangram_msgs::action::MoveArm>::SharedPtr action_move_arm_;

  std::shared_ptr<mn::CppLinuxSerial::SerialPort> arm_serial_port_;

  double j0_;
  double j1_;
  double j2_;
  double j3_;
  bool joint_states_ready_;
  std::vector<std::string> joint_names_;
  std::string serial_device_;

  void timer_callback_();

  void srv_read_position_callback_(
    const tangram_msgs::srv::ReadPosition::Request::SharedPtr request,
    tangram_msgs::srv::ReadPosition::Response response
  );

  /// \brief Determine whether to start moving
  /// \param uuid UUID of the goal
  /// \param goal Goal object of move_arm action
  /// \return Goal response object
  rclcpp_action::GoalResponse action_move_arm_goal_callback_(
    const rclcpp_action::GoalUUID & uuid,
    tangram_msgs::action::MoveArm::Goal::ConstSharedPtr goal
  );

  /// \brief Cancel the goal
  /// \param goal_handle Goal handler object of the move_arm action
  /// \return Cancel response of the object
  rclcpp_action::CancelResponse action_move_arm_cancel_callback_(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<tangram_msgs::action::MoveArm>> goal_handle
  );

  /// \brief Start moving arm
  /// \param goal_handle Goal handler of te move_arm action
  void action_move_arm_accepted_callback_(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<tangram_msgs::action::MoveArm>> goal_handle
  );

  /// \brief Move arm to the goal position
  /// \param goal_handle Goal handle object of the move_arm action
  void action_move_arm_execute_callback_(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<tangram_msgs::action::MoveArm>> goal_handle
  );

  /// \brief Get hex string of the uuid
  /// \param uuid The uuid object
  /// \return The hex string representing the uuid
  const std::string get_uuid_string(const rclcpp_action::GoalUUID & uuid);
};
}  // namespace maxarm_control
#endif  // MAXARM_CONTROL__ARM_CONTROL_HPP_
