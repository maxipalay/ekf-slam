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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(catch_ros2 REQUIRED)

  # This is the node in which integration tests occur
  add_executable(turtle_control_test
    test/turtle_control_test.cpp
  )

#   add_executable(turtle_odom_test
#     test/turtle_odom_test.cpp
#   )
  # The link libraries call links this node with catch_ros2::catch_ros2_with_node_main
  # to get the default integration test node main function
  target_link_libraries(turtle_control_test
    catch_ros2::catch_ros2_with_node_main turtlelib::turtlelib
  )
  ament_target_dependencies(turtle_control_test
    rclcpp geometry_msgs nuturtlebot_msgs sensor_msgs
  )

#   target_link_libraries(turtle_odom_test
#     catch_ros2::catch_ros2_with_node_main
#   )
#   ament_target_dependencies(turtle_odom_test
#     rclcpp std_srvs
#   )
  install(TARGETS
    turtle_control_test
    # turtle_odom_test
    DESTINATION lib/${PROJECT_NAME}
  )
  #################
  ## LAUNCH FILE ##
  #################
  # This is the launch file that will be used to run the integration test
  install(FILES
    test/turtle_control_test.launch.py
    # test/turtle_odom_test.launch.py
    DESTINATION share/${PROJECT_NAME}
  )

  ######################
  ## INTEGRATION TEST ##
  ######################
  # This CMake function allows the integration test to be run
  # when running "colcon test".
  catch_ros2_add_integration_test(turtle_control_test
    LAUNCH_FILE turtle_control_test.launch.py  # example in Python
  )
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
