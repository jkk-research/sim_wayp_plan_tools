cmake_minimum_required(VERSION 3.8)
project(sim_wayp_plan_tools)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(ament_dependencies
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_msgs
  tf2_ros
  tf2_geometry_msgs  
)

include_directories(
  include
)

add_executable(pose_arr_to_tf src/pose_arr_to_tf.cpp)
ament_target_dependencies(pose_arr_to_tf  ${ament_dependencies} )
add_executable(visuals src/visuals.cpp)
ament_target_dependencies(visuals  ${ament_dependencies} )

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

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

install(DIRECTORY 
  launch
  rviz
  DESTINATION share/${PROJECT_NAME})
install(TARGETS
  pose_arr_to_tf
  visuals
  DESTINATION lib/${PROJECT_NAME})
ament_package()