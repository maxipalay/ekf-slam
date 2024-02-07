cmake_minimum_required(VERSION 3.8)
project(nuturtle_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlelib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(nuturtle_control_interfaces REQUIRED)
find_package(std_srvs REQUIRED)


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

add_executable(turtle_control src/turtle_control.cpp)
ament_target_dependencies(turtle_control rclcpp geometry_msgs sensor_msgs nuturtlebot_msgs nav_msgs tf2 tf2_ros)
target_link_libraries(turtle_control turtlelib::turtlelib)

add_executable(odometry src/odometry.cpp)
ament_target_dependencies(odometry rclcpp geometry_msgs sensor_msgs nuturtlebot_msgs nav_msgs tf2 tf2_ros nuturtle_control_interfaces)
target_link_libraries(odometry turtlelib::turtlelib)

add_executable(circle src/circle.cpp)
ament_target_dependencies(circle rclcpp geometry_msgs std_srvs nuturtle_control_interfaces)

install(TARGETS
  turtle_control odometry circle
  DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files.
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
