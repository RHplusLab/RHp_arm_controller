cmake_minimum_required(VERSION 3.8)
project(rhparm_mtc_pick_and_place)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rhp_apriltag_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# 실행할 executable들 추가
add_executable(vision_cylinder_one src/vision_cylinder_one.cpp)
add_executable(apriltag_processor_node src/apriltag_processor_node.cpp)
add_executable(vision_cylinder_stack src/vision_cylinder_stack.cpp)
add_executable(vision_prepare src/vision_prepare.cpp)

ament_target_dependencies(
  vision_cylinder_one
  moveit_task_constructor_core
  rclcpp
  rhp_apriltag_msgs
)

ament_target_dependencies(
  apriltag_processor_node
  rclcpp
  rhp_apriltag_msgs
)
ament_target_dependencies(
  vision_cylinder_stack
  moveit_task_constructor_core
  rclcpp
  rhp_apriltag_msgs
  std_msgs
)

ament_target_dependencies(
  vision_prepare
  moveit_task_constructor_core
)

install(TARGETS vision_cylinder_one
DESTINATION lib/${PROJECT_NAME})
install(TARGETS vision_cylinder_stack
DESTINATION lib/${PROJECT_NAME})
install(TARGETS apriltag_processor_node
DESTINATION lib/${PROJECT_NAME})
install(TARGETS vision_prepare
DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
# the following line skips the linter which checks for copyrights
# uncomment the line when a copyright and license is not present in all source files
#set(ament_cmake_copyright_FOUND TRUE)
# the following line skips cpplint (only works in a git repo)
# uncomment the line when this package is not in a git repo
#set(ament_cmake_cpplint_FOUND TRUE)
ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
