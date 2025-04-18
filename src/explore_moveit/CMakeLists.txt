cmake_minimum_required(VERSION 3.8)
project(explore_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(my_test_cpp src/my_test_cpp.cpp)
ament_target_dependencies(my_test_cpp rclcpp)

add_executable(try_control_moveit src/try_control_moveit.cpp)
ament_target_dependencies(try_control_moveit rclcpp moveit_ros_planning_interface)

add_executable(control_playground src/control_playground.cpp)
ament_target_dependencies(control_playground rclcpp moveit_ros_planning_interface)

install(TARGETS my_test_cpp
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS try_control_moveit
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS control_playground
  DESTINATION lib/${PROJECT_NAME}
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

# Install Python node
install(PROGRAMS
  scripts/my_test_py.py
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/robot_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
