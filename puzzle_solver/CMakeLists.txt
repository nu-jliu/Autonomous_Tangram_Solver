cmake_minimum_required(VERSION 3.8)
project(puzzle_solver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tangram_msgs REQUIRED)
find_package(OpenCV 4.5 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem thread)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(tangram_match src/tangram_match.cpp)
target_include_directories(tangram_match PRIVATE include)
ament_target_dependencies(tangram_match rclcpp)

add_executable(puzzle_solver src/puzzle_solver.cpp)
target_include_directories(puzzle_solver PRIVATE include)
target_link_libraries(puzzle_solver 
  ${OpenCV_LIBS}
  tangram_match
)
ament_target_dependencies(puzzle_solver 
  rclcpp
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
  cv_bridge
  sensor_msgs
  tangram_msgs
)

install(TARGETS tangram_match
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS 
  puzzle_solver
  tangram_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
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
