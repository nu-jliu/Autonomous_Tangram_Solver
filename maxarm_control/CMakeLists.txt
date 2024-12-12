cmake_minimum_required(VERSION 3.8)
project(maxarm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tangram_msgs REQUIRED)
find_package(CppLinuxSerial REQUIRED)
find_package(Armadillo REQUIRED)

include_directories(${ARMADILLO_INCLUDE_DIRS})

add_executable(robot_control src/robot_control.cpp)
target_include_directories(robot_control PRIVATE include)
target_link_libraries(robot_control 
  CppLinuxSerial::CppLinuxSerial
)
ament_target_dependencies(robot_control
  rclcpp
  rclcpp_action
  std_srvs
  sensor_msgs
  tangram_msgs
)

add_executable(frame_align src/frame_align.cpp)
target_include_directories(frame_align PRIVATE include)
target_link_libraries(frame_align ${ARMADILLO_LIBRARIES})
ament_target_dependencies(frame_align 
  rclcpp
  tangram_msgs
)

install(TARGETS
  robot_control
  frame_align
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  config 
  launch
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

# Building documentation should be optional.
# To build documentation pass -DBUILD_DOCS=ON when generating the build system
option(BUILD_DOCS "Build the documentation" OFF)

# build just because Doxygen is missing
if (BUILD_DOCS)
  find_package(Doxygen)
    
  if (DOXYGEN_FOUND)
    set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in)
    set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)

    set(DOXYGEN_USE_MDFILE_AS_MAINPAGE README.md)

    configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)

    add_custom_target(doc_doxygen ALL
      COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      COMMENT "Generating API documentation with Doxygen"
      VERBATIM
    )
  endif(DOXYGEN_FOUND)
endif(BUILD_DOCS)

ament_package()
