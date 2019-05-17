# Install script for directory: /home/arlpx/Documents/SailMAVpx/SailMAV/src/lib/drivers

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
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/lib/drivers/accelerometer/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/lib/drivers/airspeed/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/lib/drivers/device/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/lib/drivers/gyroscope/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/lib/drivers/led/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/lib/drivers/linux_gpio/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/lib/drivers/magnetometer/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/lib/drivers/smbus/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/lib/drivers/tone_alarm/cmake_install.cmake")

endif()
