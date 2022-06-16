# CMake generated Testfile for 
# Source directory: /home/sahil/DorleCo/src/carla-ros-bridge/catkin_ws/src/ros-bridge/carla_ackermann_control
# Build directory: /home/sahil/DorleCo/build/carla_ackermann_control
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_carla_ackermann_control_roslaunch-check_launch "/home/sahil/DorleCo/build/carla_ackermann_control/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/sahil/DorleCo/build/carla_ackermann_control/test_results/carla_ackermann_control/roslaunch-check_launch.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/sahil/DorleCo/build/carla_ackermann_control/test_results/carla_ackermann_control" "/opt/ros/noetic/share/roslaunch/cmake/../scripts/roslaunch-check -o \"/home/sahil/DorleCo/build/carla_ackermann_control/test_results/carla_ackermann_control/roslaunch-check_launch.xml\" \"/home/sahil/DorleCo/src/carla-ros-bridge/catkin_ws/src/ros-bridge/carla_ackermann_control/launch\" ")
set_tests_properties(_ctest_carla_ackermann_control_roslaunch-check_launch PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/roslaunch/cmake/roslaunch-extras.cmake;66;catkin_run_tests_target;/home/sahil/DorleCo/src/carla-ros-bridge/catkin_ws/src/ros-bridge/carla_ackermann_control/CMakeLists.txt;14;roslaunch_add_file_check;/home/sahil/DorleCo/src/carla-ros-bridge/catkin_ws/src/ros-bridge/carla_ackermann_control/CMakeLists.txt;0;")
subdirs("gtest")
