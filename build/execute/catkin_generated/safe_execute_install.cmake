execute_process(COMMAND "/home/khoixx/dev_ros1_ws/build/execute/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/khoixx/dev_ros1_ws/build/execute/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
