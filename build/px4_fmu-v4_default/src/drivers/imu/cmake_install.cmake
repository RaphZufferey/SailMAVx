# Install script for directory: /home/arlpx/Documents/SailMAVpx/SailMAV/src/drivers/imu

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
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/imu/adis16448/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/imu/adis16477/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/imu/adis16497/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/imu/bma180/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/imu/bmi055/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/imu/bmi160/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/imu/fxas21002c/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/imu/fxos8701cq/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/imu/icm20948/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/imu/l3gd20/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/imu/lsm303d/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/imu/mpu6000/cmake_install.cmake")
  include("/home/arlpx/Documents/SailMAVpx/SailMAV/build/px4_fmu-v4_default/src/drivers/imu/mpu9250/cmake_install.cmake")

endif()

