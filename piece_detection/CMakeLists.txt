cmake_minimum_required(VERSION 3.8)
project(piece_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-O3 -Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tangram_msgs REQUIRED)
find_package(OpenCV 4.5 REQUIRED)
find_package(tangram_utils REQUIRED)
find_package(uuid_v4 REQUIRED)
find_package(realsense2 REQUIRED)
find_package(Boost REQUIRED)

include_directories(
  ${realsense2_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(piece_detection src/piece_detection.cpp)
target_include_directories(piece_detection PRIVATE include)
target_link_libraries(piece_detection
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  tangram_utils::tangram_utils
  uuid_v4::uuid_v4
)
ament_target_dependencies(piece_detection 
  rclcpp
  std_srvs
  cv_bridge
  sensor_msgs
  tangram_msgs
)

add_executable(rs_pixel_to_real src/rs_pixel_to_real.cpp)
target_include_directories(rs_pixel_to_real PRIVATE include)
target_link_libraries(rs_pixel_to_real ${realsense2_LIBRARY})
ament_target_dependencies(rs_pixel_to_real 
  rclcpp
  std_srvs
  tf2_ros
  cv_bridge
  sensor_msgs
  tangram_msgs
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS 
  piece_detection
  rs_pixel_to_real
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
