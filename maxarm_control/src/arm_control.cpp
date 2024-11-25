#include <memory>
#include <chrono>

#include "maxarm_control/arm_control.hpp"

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

namespace maxarm_control
{
using namespace std::chrono_literals;

ArmControl::ArmControl()
: rclcpp::Node("arm_control"),
  j0_(0.0), j1_(0.0), j2_(0.0), j3_(0.0), joint_states_ready_(false),
  joint_names_{"base", "shouler", "elbow", "wrist"}
{
  rcl_interfaces::msg::ParameterDescriptor serial_device_des;

  serial_device_des.description = "Name of the serial device";

  declare_parameter<std::string>("serial_device", "/dev/ttyUSB0", serial_device_des);

  serial_device_ = get_parameter("serial_device").as_string();

  arm_serial_port_ = std::make_shared<mn::CppLinuxSerial::SerialPort>(
    serial_device_,
    mn::CppLinuxSerial::BaudRate::B_115200
  );

  loop_rate_ = std::make_shared<rclcpp::Rate>(100ms);
}

void ArmControl::timer_callback_()
{
  if (joint_states_ready_) {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = get_clock()->now();
    msg.position = {j0_, j1_, j2_, j3_};
    msg.name = joint_names_;

    pub_arm_joint_states_->publish(msg);
  }
}

rclcpp_action::GoalResponse ArmControl::action_move_arm_goal_callback_(
  const rclcpp_action::GoalUUID & uuid,
  tangram_msgs::action::MoveArm::Goal::ConstSharedPtr goal
)
{
  const double x = goal->goal.position.x;
  const double y = goal->goal.position.y;
  const double z = goal->goal.position.z;

  const std::string uuid_str = get_uuid_string(uuid);

  RCLCPP_INFO_STREAM(
    get_logger(),
    "Received request with uuid " << uuid_str <<
      " to move MaxArm to position: " << x << ", " << y << ", " << z);

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ArmControl::action_move_arm_cancel_callback_(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<tangram_msgs::action::MoveArm>> goal_handle
)
{
  const rclcpp_action::GoalUUID goal_uuid = goal_handle->get_goal_id();
  const std::string uuid_str = get_uuid_string(goal_uuid);

  RCLCPP_INFO_STREAM(get_logger(), "Received the request to cancel goal with uuid " << uuid_str);

  return rclcpp_action::CancelResponse::ACCEPT;
}

void ArmControl::action_move_arm_accepted_callback_(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<tangram_msgs::action::MoveArm>> goal_handle
)
{
  std::thread execute_thread(
    std::bind(&ArmControl::action_move_arm_execute_callback_, this, std::placeholders::_1),
    goal_handle
  );

  execute_thread.detach();
}

void ArmControl::action_move_arm_execute_callback_(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<tangram_msgs::action::MoveArm>> goal_handle
)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<tangram_msgs::action::MoveArm::Result>();

  const double x = goal->goal.position.x;
  const double y = goal->goal.position.y;
  const double z = goal->goal.position.z;

  std::stringstream ss_command("");
  ss_command << "arm.set_position((";
  ss_command << x << ", " << y << ", " << z;
  ss_command << "), 3000)\r\n";

  arm_serial_port_->Write(ss_command.str());

  while (rclcpp::ok() && arm_serial_port_->Available() < 1) {
    RCLCPP_WARN_STREAM(get_logger(), "Waiting for response");
  }

  std::this_thread::sleep_for(100ms);

  std::string response_message;
  arm_serial_port_->Read(response_message);

  if (rclcpp::ok()) {
    result->success = true;
    goal_handle->succeed(result);

    RCLCPP_INFO_STREAM(get_logger(), "MaxArm moved to " << x << ", " << y << ", " << z);
  }
}

const std::string ArmControl::get_uuid_string(const rclcpp_action::GoalUUID & uuid)
{
  std::stringstream ss;

  ss << std::hex;
  ss << std::setw(2);
  ss << std::setfill('0');

  for (uint8_t num : uuid) {
    const auto val = static_cast<int>(num);
    ss << val;
  }

  return ss.str();
}
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<maxarm_control::ArmControl>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
