# CMake generated Testfile for 
# Source directory: /home/sahil/carla-ros-bridge/catkin_ws/src/ros-bridge/carla_waypoint_publisher
# Build directory: /home/sahil/carla-ros-bridge/catkin_ws/build/carla_waypoint_publisher
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_carla_waypoint_publisher_roslaunch-check_launch "/home/sahil/carla-ros-bridge/catkin_ws/build/carla_waypoint_publisher/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/sahil/carla-ros-bridge/catkin_ws/build/carla_waypoint_publisher/test_results/carla_waypoint_publisher/roslaunch-check_launch.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/sahil/carla-ros-bridge/catkin_ws/build/carla_waypoint_publisher/test_results/carla_waypoint_publisher" "/opt/ros/noetic/share/roslaunch/cmake/../scripts/roslaunch-check -o \"/home/sahil/carla-ros-bridge/catkin_ws/build/carla_waypoint_publisher/test_results/carla_waypoint_publisher/roslaunch-check_launch.xml\" \"/home/sahil/carla-ros-bridge/catkin_ws/src/ros-bridge/carla_waypoint_publisher/launch\" ")
set_tests_properties(_ctest_carla_waypoint_publisher_roslaunch-check_launch PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/roslaunch/cmake/roslaunch-extras.cmake;66;catkin_run_tests_target;/home/sahil/carla-ros-bridge/catkin_ws/src/ros-bridge/carla_waypoint_publisher/CMakeLists.txt;15;roslaunch_add_file_check;/home/sahil/carla-ros-bridge/catkin_ws/src/ros-bridge/carla_waypoint_publisher/CMakeLists.txt;0;")
subdirs("gtest")
