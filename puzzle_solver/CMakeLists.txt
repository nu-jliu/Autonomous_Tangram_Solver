cmake_minimum_required(VERSION 3.8)
project(puzzle_solver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tangram_msgs REQUIRED)
find_package(tangram_utils REQUIRED)
find_package(OpenCV 4.5 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(puzzle_solver src/puzzle_solver.cpp)
target_include_directories(puzzle_solver PRIVATE include)
target_link_libraries(puzzle_solver 
  ${OpenCV_LIBS}
  tangram_utils::tangram_utils
)
ament_target_dependencies(puzzle_solver 
  rclcpp
  std_srvs
  cv_bridge
  sensor_msgs
  tangram_msgs
)

add_executable(tangram_publisher src/tangram_publisher.cpp)
target_include_directories(tangram_publisher PRIVATE include)
target_link_libraries(tangram_publisher 
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
)
ament_target_dependencies(tangram_publisher
  rclcpp
  std_srvs
  cv_bridge
  sensor_msgs
  tangram_msgs
)

add_executable(solution_pixel_to_real src/solution_pixel_to_real.cpp)
target_include_directories(solution_pixel_to_real PRIVATE include)
ament_target_dependencies(solution_pixel_to_real 
  rclcpp 
  std_srvs
  rcl_interfaces 
  tangram_msgs
)

add_executable(svg_generator src/svg_generator.cpp)
target_include_directories(svg_generator PRIVATE include)
target_link_libraries(svg_generator
  ${Boost_LIBRARIES}
)
ament_target_dependencies(svg_generator
  rclcpp
  std_srvs
  rcl_interfaces
  tangram_msgs
)

add_executable(web_camera src/web_camera.cpp)
target_include_directories(web_camera PRIVATE include)
target_link_libraries(web_camera
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
)
ament_target_dependencies(web_camera
  rclcpp
  cv_bridge
  rcl_interfaces
  sensor_msgs
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS 
  puzzle_solver
  tangram_publisher
  solution_pixel_to_real
  svg_generator
  web_camera
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  config
  launch
  images
  model
  DESTINATION share/${PROJECT_NAME}
)

# ament_python_install_package(${PROJECT_NAME})

# install(PROGRAMS 
#     ${PROJECT_NAME}/tangram_segment.py
#     ${PROJECT_NAME}/network.py
#     DESTINATION lib/${PROJECT_NAME}
# )

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
