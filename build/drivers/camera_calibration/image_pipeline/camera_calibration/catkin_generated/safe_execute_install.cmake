execute_process(COMMAND "/home/ben/gripper_proj_py/build/drivers/camera_calibration/image_pipeline/camera_calibration/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/ben/gripper_proj_py/build/drivers/camera_calibration/image_pipeline/camera_calibration/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
