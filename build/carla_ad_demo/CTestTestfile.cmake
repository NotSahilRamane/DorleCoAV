# CMake generated Testfile for 
# Source directory: /home/sahil/DorleCo/src/carla-ros-bridge/catkin_ws/src/ros-bridge/carla_ad_demo
# Build directory: /home/sahil/DorleCo/build/carla_ad_demo
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_carla_ad_demo_roslaunch-check_launch "/home/sahil/DorleCo/build/carla_ad_demo/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/sahil/DorleCo/build/carla_ad_demo/test_results/carla_ad_demo/roslaunch-check_launch.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/sahil/DorleCo/build/carla_ad_demo/test_results/carla_ad_demo" "/opt/ros/noetic/share/roslaunch/cmake/../scripts/roslaunch-check -o \"/home/sahil/DorleCo/build/carla_ad_demo/test_results/carla_ad_demo/roslaunch-check_launch.xml\" \"/home/sahil/DorleCo/src/carla-ros-bridge/catkin_ws/src/ros-bridge/carla_ad_demo/launch\" ")
set_tests_properties(_ctest_carla_ad_demo_roslaunch-check_launch PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/roslaunch/cmake/roslaunch-extras.cmake;66;catkin_run_tests_target;/home/sahil/DorleCo/src/carla-ros-bridge/catkin_ws/src/ros-bridge/carla_ad_demo/CMakeLists.txt;12;roslaunch_add_file_check;/home/sahil/DorleCo/src/carla-ros-bridge/catkin_ws/src/ros-bridge/carla_ad_demo/CMakeLists.txt;0;")
subdirs("gtest")
