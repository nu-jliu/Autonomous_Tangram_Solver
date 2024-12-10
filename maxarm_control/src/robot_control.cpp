#include <memory>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

#include "maxarm_control/robot_control.hpp"

#include <rcl_interfaces/msg/parameter_descriptor.hpp>

namespace maxarm_control
{
using namespace std::chrono_literals;

RobotControl::RobotControl()
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
    mn::CppLinuxSerial::BaudRate::B_115200,
    mn::CppLinuxSerial::NumDataBits::EIGHT,
    mn::CppLinuxSerial::Parity::NONE,
    mn::CppLinuxSerial::NumStopBits::ONE
  );
  arm_serial_port_->Open();

  loop_rate_ = std::make_shared<rclcpp::Rate>(100ms);

  arm_serial_port_->Write("\r\n");
  wait_for_data_();

  go_home_();
  nozzle_off_();
  run_buzzer_();

  srv_grasp_ = create_service<std_srvs::srv::Trigger>(
    "gripper/grasp",
    std::bind(
      &RobotControl::srv_grasp_callback_,
      this,
      std::placeholders::_1,
      std::placeholders::_2
    ),
    10
  );

  srv_release_ = create_service<std_srvs::srv::Trigger>(
    "gripper/release",
    std::bind(
      &RobotControl::srv_release_callback_,
      this,
      std::placeholders::_1,
      std::placeholders::_2
    ),
    10
  );

  action_move_arm_ = rclcpp_action::create_server<tangram_msgs::action::MoveArm>(
    this,
    "move_arm",
    std::bind(
      &RobotControl::action_move_arm_goal_callback_,
      this,
      std::placeholders::_1,
      std::placeholders::_2
    ),
    std::bind(
      &RobotControl::action_move_arm_cancel_callback_,
      this,
      std::placeholders::_1
    ),
    std::bind(
      &RobotControl::action_move_arm_accepted_callback_,
      this,
      std::placeholders::_1
    )
  );

  action_go_home_ = rclcpp_action::create_server<tangram_msgs::action::GoHome>(
    this,
    "go_home",
    std::bind(
      &RobotControl::action_go_home_goal_callback_,
      this,
      std::placeholders::_1,
      std::placeholders::_2
    ),
    std::bind(
      &RobotControl::action_go_home_cancel_callback_,
      this,
      std::placeholders::_1
    ),
    std::bind(
      &RobotControl::action_go_home_accepted_callback_,
      this,
      std::placeholders::_1
    )
  );
}

RobotControl::~RobotControl()
{
  RCLCPP_INFO(get_logger(), "Going home");
  go_home_();

  RCLCPP_INFO(get_logger(), "Turning off nozzle");
  nozzle_off_();

  RCLCPP_INFO_STREAM(get_logger(), "Closing serial port " << serial_device_);
  arm_serial_port_->Close();

  rclcpp::shutdown();
}

void RobotControl::timer_callback_()
{
  if (joint_states_ready_) {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = get_clock()->now();
    msg.position = {j0_, j1_, j2_, j3_};
    msg.name = joint_names_;

    pub_arm_joint_states_->publish(msg);
  }
}

void RobotControl::srv_grasp_callback_(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response
)
{
  (void) request;
  nozzle_on_();

  response->success = true;
}

void RobotControl::srv_release_callback_(
  const std_srvs::srv::Trigger::Request::SharedPtr request,
  std_srvs::srv::Trigger::Response::SharedPtr response
)
{
  (void) request;
  nozzle_off_();

  response->success = true;
}

rclcpp_action::GoalResponse RobotControl::action_move_arm_goal_callback_(
  const rclcpp_action::GoalUUID & uuid,
  tangram_msgs::action::MoveArm::Goal::ConstSharedPtr goal
)
{
  const bool pick = goal->pick;
  const double x = goal->goal.x;
  const double y = goal->goal.y;
  const double z = goal->goal.z;

  const std::string uuid_str = get_uuid_string(uuid);

  if (pick) {
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Received request with uuid " << uuid_str <<
        " to move MaxArm pick at position: " << x << ", " << y << ", " << z
    );
  } else {
    RCLCPP_INFO_STREAM(
      get_logger(),
      "Received request with uuid " << uuid_str <<
        " to move MaxArm place at position: " << x << ", " << y << ", " << z
    );
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RobotControl::action_move_arm_cancel_callback_(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<tangram_msgs::action::MoveArm>> goal_handle
)
{
  const rclcpp_action::GoalUUID goal_uuid = goal_handle->get_goal_id();
  const std::string uuid_str = get_uuid_string(goal_uuid);

  RCLCPP_INFO_STREAM(get_logger(), "Received the request to cancel goal with uuid " << uuid_str);

  return rclcpp_action::CancelResponse::ACCEPT;
}

void RobotControl::action_move_arm_accepted_callback_(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<tangram_msgs::action::MoveArm>> goal_handle
)
{
  std::thread execute_thread(
    std::bind(&RobotControl::action_move_arm_execute_callback_, this, std::placeholders::_1),
    goal_handle
  );

  execute_thread.detach();
}

void RobotControl::action_move_arm_execute_callback_(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<tangram_msgs::action::MoveArm>> goal_handle
)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<tangram_msgs::action::MoveArm::Result>();

  const bool pick = goal->pick;
  const double x = goal->goal.x * 1000.0;
  const double y = goal->goal.y * 1000.0;
  const double z = goal->goal.z * 1000.0;

  std::stringstream ss_arm("");
  ss_arm << "arm.set_position((";
  ss_arm << x << ", " << y << ", " << z;
  ss_arm << "), 1200)\r\n";

  arm_serial_port_->Write(ss_arm.str());
  const bool success = wait_for_data_();

  if (rclcpp::ok() & !success) {
    result->success = success;
    goal_handle->abort(result);

    RCLCPP_ERROR(get_logger(), "MaxArm moving failed");
    return;
  }

  const double radian = std::atan2(-x, -y);

  double degree = radian * 180.0 / M_PI;
  if (pick) {
    degree += 90;
  }

  std::stringstream ss_nozzle("");
  ss_nozzle << "nozzle.set_angle(";
  ss_nozzle << degree;
  ss_nozzle << ")\r\n";

  arm_serial_port_->Write(ss_nozzle.str());
  wait_for_data_();

  std::this_thread::sleep_for(1.5s);

  if (rclcpp::ok()) {
    result->success = success;
    goal_handle->succeed(result);

    RCLCPP_INFO_STREAM(get_logger(), "MaxArm moved to " << x << ", " << y << ", " << z);
  }
}

rclcpp_action::GoalResponse RobotControl::action_go_home_goal_callback_(
  const rclcpp_action::GoalUUID & uuid,
  tangram_msgs::action::GoHome::Goal::ConstSharedPtr goal
)
{
  (void) goal;
  const std::string uuid_str = get_uuid_string(uuid);

  RCLCPP_INFO_STREAM(
    get_logger(),
    "Received request with uuid " << uuid_str << " to go home"
  );

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RobotControl::action_go_home_cancel_callback_(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<tangram_msgs::action::GoHome>> goal_handle
)
{
  const rclcpp_action::GoalUUID goal_uuid = goal_handle->get_goal_id();
  const std::string uuid_str = get_uuid_string(goal_uuid);

  RCLCPP_INFO_STREAM(get_logger(), "Received the request to cancel goal with uuid " << uuid_str);

  return rclcpp_action::CancelResponse::ACCEPT;
}

void RobotControl::action_go_home_accepted_callback_(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<tangram_msgs::action::GoHome>> goal_handle
)
{
  std::thread execute_thread(
    std::bind(&RobotControl::action_go_home_execute_callback_, this, std::placeholders::_1),
    goal_handle
  );

  execute_thread.detach();
}

void RobotControl::action_go_home_execute_callback_(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<tangram_msgs::action::GoHome>> goal_handle
)
{
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<tangram_msgs::action::GoHome::Result>();

  go_home_();

  if (rclcpp::ok()) {
    result->success = true;
    goal_handle->succeed(result);

    RCLCPP_INFO_STREAM(get_logger(), "MaxArm moved to home");
  }
}

const std::string RobotControl::get_uuid_string(const rclcpp_action::GoalUUID & uuid)
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

void RobotControl::go_home_()
{
  arm_serial_port_->Write("arm.go_home(1000)\r\n");
  wait_for_data_();

  arm_serial_port_->Write("nozzle.set_angle(0)\r\n");
  wait_for_data_();

  std::this_thread::sleep_for(1.0s);
}

bool RobotControl::wait_for_data_()
{
  while (rclcpp::ok()) {
    if (arm_serial_port_->Available() > 0) {
      break;
    } else {
      RCLCPP_WARN(get_logger(), "Data not available, waiting");
    }

    loop_rate_->sleep();
  }

  std::this_thread::sleep_for(200ms);

  std::string response_message;
  arm_serial_port_->Read(response_message);

  std::istringstream ss_response(response_message);
  std::string line;

  bool success = false;

  RCLCPP_INFO(get_logger(), "Received response:");
  while (std::getline(ss_response, line)) {
    RCLCPP_INFO_STREAM(get_logger(), line);
    if (line.substr(0, 4) == "True") {
      success = true;
    }
  }
  RCLCPP_INFO_STREAM(get_logger(), " ");

  return success;
}

void RobotControl::run_buzzer_()
{
  for (int i = 0; i < 3; ++i) {
    arm_serial_port_->Write("buzzer.on()\r\n");
    wait_for_data_();

    arm_serial_port_->Write("buzzer.off()\r\n");
    wait_for_data_();

    std::this_thread::sleep_for(50ms);
  }
}

void RobotControl::nozzle_on_()
{
  arm_serial_port_->Write("nozzle.on()\r\n");
  wait_for_data_();
}

void RobotControl::nozzle_off_()
{
  arm_serial_port_->Write("nozzle.off()\r\n");
  wait_for_data_();
}
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<maxarm_control::RobotControl>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
