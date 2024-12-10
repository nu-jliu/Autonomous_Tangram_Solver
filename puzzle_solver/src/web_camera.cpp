#include <memory>
#include <chrono>
#include <regex>
#include <cstdlib>
#include <cstdio>
#include <algorithm>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include "puzzle_solver/web_camera.hpp"


namespace puzzle_solver
{
using namespace std::chrono_literals;
WebCamera::WebCamera()
: Node("web_camera")
{
  // Declare the camera name parameter
  rcl_interfaces::msg::ParameterDescriptor camera_name_des;

  camera_name_des.description = "Name of the camera device";

  declare_parameter<std::string>("camera_name", "Amcrest AWC2198 USB Webcam", camera_name_des);

  camera_name_ = get_parameter("camera_name").as_string();

  // List devices in /dev directory using Boost Filesystem
  std::vector<std::string> devices;
  for (const auto & entry : fs::directory_iterator("/dev")) {
    if (entry.path().filename().string().find("video") != std::string::npos) {
      devices.push_back(entry.path().string());
    }
  }

  std::vector<std::pair<std::string, std::string>> device_info;

  // Get device information using v4l2-ctl
  for (const auto & device : devices) {
    try {
      std::string command = "v4l2-ctl --device=" + device + " --info";
      FILE * pipe = popen(command.c_str(), "r");
      if (!pipe) {
        throw std::runtime_error("Failed to run v4l2-ctl");
      }

      char buffer[128];
      std::string result;
      while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        result += buffer;
      }

      pclose(pipe);
      // RCLCPP_INFO_STREAM(get_logger(), result);

      std::stringstream ss(result);
      std::string line;

      while (std::getline(ss, line)) {
        if (!line.empty()) {

          const size_t index = line.find("Card type");
          if (index != std::string::npos) {
            RCLCPP_INFO_STREAM(get_logger(), line);
            const std::string name = line.substr(index + 10ul, line.length());
            device_info.emplace_back(device, name);
          }
        }
      }

      // std::regex card_type_regex("Card type:\\s*(.+)");
      // std::smatch match;
      // if (std::regex_search(result, match, card_type_regex)) {
      //   std::string name = match[1];
      //   device_info.emplace_back(device, name);
      // }
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(get_logger(), "Failed to get info for " << device << ": " << e.what());
    }
  }

  std::vector<int> device_numbers;

  // Filter devices matching the camera name
  for (const auto &[device, name] : device_info) {
    RCLCPP_INFO_STREAM(get_logger(), "Device: " << device << ", Name: " << name);
    if (name.find(camera_name_) != std::string::npos) {
      std::string num_str = device.substr(device.find_last_of("video") + 1);
      RCLCPP_INFO_STREAM(get_logger(), "Number: " << num_str);
      device_numbers.push_back(std::stoi(num_str));
    }
  }

  if (device_numbers.empty()) {
    RCLCPP_ERROR(this->get_logger(), "No device found with the specified camera name.");
    rclcpp::shutdown();
    return;
  }

  device_ = "/dev/video" + std::to_string(
    *std::min_element(
      device_numbers.begin(),
      device_numbers.end()
    )
  );

  // Open the video device
  cap_.open(device_);

  if (cap_.isOpened()) {
    RCLCPP_INFO(this->get_logger(), "Opened device: %s", device_.c_str());
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to open device: %s", device_.c_str());
    rclcpp::shutdown();
    return;
  }

  // Set up the CvBridge and publisher
  pub_webcam_image_ = create_publisher<sensor_msgs::msg::Image>(
    "puzzle/camera/color/image_raw",
    10
  );

  // Create a timer for periodic image capture and publishing
  timer_ = create_wall_timer(
    0.05s,
    std::bind(
      &WebCamera::timer_callback,
      this
    )
  );
}

WebCamera::~WebCamera()
{
  if (cap_.isOpened()) {
    RCLCPP_INFO(get_logger(), "Shutting down camera");
    cap_.release();
  }

  rclcpp::shutdown();
}

void WebCamera::timer_callback()
{
  try {
    cv::Mat frame;

    if (cap_.read(frame)) {
      const auto image_msg = cv_bridge::CvImage(
        std_msgs::msg::Header(),
        sensor_msgs::image_encodings::BGR8,
        frame
      ).toImageMsg();
      pub_webcam_image_->publish(*image_msg);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
  }
}
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<puzzle_solver::WebCamera>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
