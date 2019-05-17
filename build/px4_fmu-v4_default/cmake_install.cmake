# Install script for directory: /home/arlpx/Documents/SailMAVpx/SailMAV

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "MinSizeRel")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/lib/DriverFramework/framework/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/barometer/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/batt_smbus/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/camera_capture/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/camera_trigger/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/differential_pressure/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/distance_sensor/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/gps/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/heater/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/imu/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/irlock/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/lights/blinkm/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/lights/oreoled/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/lights/rgbled/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/lights/rgbled_ncp5623c/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/magnetometer/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/mkblctrl/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/pca9685/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/pmw3901/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/pwm_input/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/pwm_out_sim/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/px4flow/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/px4fmu/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/rc_input/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/stm32/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/stm32/adc/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/stm32/tone_alarm/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/tap_esc/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/telemetry/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/test_ppm/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/tone_alarm/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/uavcan/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/as5048b/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/modules/attitude_estimator_q/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/modules/camera_feedback/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/modules/commander/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/modules/dataman/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/modules/ekf2/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/modules/events/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/modules/fw_att_control/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/modules/fw_pos_control_l1/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/modules/gnd_att_control/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/modules/gnd_pos_control/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/modules/land_detector/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/modules/landing_target_estimator/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/modules/load_mon/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/modules/local_position_estimator/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/modules/logger/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/modules/mavlink/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/modules/mc_att_control/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/modules/mc_pos_control/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/modules/navigator/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/modules/sensors/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/modules/sih/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/modules/vmount/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/modules/vtol_att_control/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/modules/wind_estimator/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/modules/sailing/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/systemcmds/bl_update/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/systemcmds/config/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/systemcmds/dumpfile/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/systemcmds/esc_calib/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/systemcmds/hardfault_log/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/systemcmds/led_control/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/systemcmds/mixer/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/systemcmds/motor_ramp/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/systemcmds/motor_test/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/systemcmds/mtd/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/systemcmds/nshterm/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/systemcmds/param/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/systemcmds/perf/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/systemcmds/pwm/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/systemcmds/reboot/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/systemcmds/reflect/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/systemcmds/sd_bench/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/systemcmds/shutdown/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/systemcmds/tests/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/systemcmds/top/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/systemcmds/topic_listener/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/systemcmds/tune_control/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/systemcmds/usb_connected/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/systemcmds/ver/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/examples/bottle_drop/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/examples/fixedwing_control/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/examples/hello/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/examples/hwtest/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/examples/position_estimator_inav/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/examples/px4_mavlink_debug/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/examples/px4_simple_app/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/examples/rover_steering_control/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/examples/segway/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/examples/uuv_example_app/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/boards/px4/fmu-v4/src/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/platforms/nuttx/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")