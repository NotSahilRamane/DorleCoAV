execute_process(COMMAND "/home/sahil/DorleCo/build/carla_common/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/sahil/DorleCo/build/carla_common/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
