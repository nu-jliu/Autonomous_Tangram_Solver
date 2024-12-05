#ifndef MAXARM_CONTROL__ROBOT_CONTROL_HPP___
#define MAXARM_CONTROL__ROBOT_CONTROL_HPP___

#include <CppLinuxSerial/SerialPort.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <std_srvs/srv/trigger.hpp>

#include "tangram_msgs/action/move_arm.hpp"
#include "tangram_msgs/action/go_home.hpp"
#include "tangram_msgs/srv/read_position.hpp"

namespace maxarm_control
{
class RobotControl : public rclcpp::Node
{
public:
  RobotControl();
  virtual ~RobotControl();

private:
  rclcpp::Rate::SharedPtr loop_rate_;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Service<tangram_msgs::srv::ReadPosition>::SharedPtr srv_read_position_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_grasp_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_release_;

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_arm_joint_states_;

  rclcpp_action::Server<tangram_msgs::action::MoveArm>::SharedPtr action_move_arm_;
  rclcpp_action::Server<tangram_msgs::action::GoHome>::SharedPtr action_go_home_;

  std::shared_ptr<mn::CppLinuxSerial::SerialPort> arm_serial_port_;

  double j0_;
  double j1_;
  double j2_;
  double j3_;
  bool joint_states_ready_;
  std::vector<std::string> joint_names_;
  std::string serial_device_;

  /// \brief Timer callback of the node
  void timer_callback_();

  /// \brief Read position of the robot arm
  /// \param request Request object of the read_position service
  /// \param response Response object of the read_position service
  void srv_read_position_callback_(
    const tangram_msgs::srv::ReadPosition::Request::SharedPtr request,
    tangram_msgs::srv::ReadPosition::Response response
  );

  void srv_grasp_callback_(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response
  );

  void srv_release_callback_(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response
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

  /// \brief Determine whether to start moving
  /// \param uuid UUID of the goal
  /// \param goal Goal object of go_home action
  /// \return Goal response object
  rclcpp_action::GoalResponse action_go_home_goal_callback_(
    const rclcpp_action::GoalUUID & uuid,
    tangram_msgs::action::GoHome::Goal::ConstSharedPtr goal
  );

  /// \brief Cancel the goal
  /// \param goal_handle Goal handler object of the go_home action
  /// \return Cancel response of the object
  rclcpp_action::CancelResponse action_go_home_cancel_callback_(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<tangram_msgs::action::GoHome>> goal_handle
  );

  /// \brief Start moving arm
  /// \param goal_handle Goal handler of te go_home action
  void action_go_home_accepted_callback_(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<tangram_msgs::action::GoHome>> goal_handle
  );

  /// \brief Move arm to home position
  /// \param goal_handle Goal handle object of the go_home action
  void action_go_home_execute_callback_(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<tangram_msgs::action::GoHome>> goal_handle
  );

  /// \brief Get hex string of the uuid
  /// \param uuid The uuid object
  /// \return The hex string representing the uuid
  const std::string get_uuid_string(const rclcpp_action::GoalUUID & uuid);

  /// \brief Wait until data is available
  /// \return If the action is success
  bool wait_for_data_();

  /// \brief Move the robot arm to home position
  void go_home_();

  /// \brief Sound the buzzer for 3 times
  void run_buzzer_();

  /// \brief Turn off the nozzle
  void nozzle_off_();
};
}  // namespace maxarm_control
#endif  // MAXARM_CONTROL__ROBOT_CONTROL_HPP___
