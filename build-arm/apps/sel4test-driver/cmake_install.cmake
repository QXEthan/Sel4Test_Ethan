# Install script for directory: /home/ethan/sel4test_ethan/projects/sel4test/apps/sel4test-driver

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
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "TRUE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/aarch64-linux-gnu-objdump")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/ethan/sel4test_ethan/build-arm/apps/sel4test-driver/sel4runtime/cmake_install.cmake")
  include("/home/ethan/sel4test_ethan/build-arm/apps/sel4test-driver/musllibc/cmake_install.cmake")
  include("/home/ethan/sel4test_ethan/build-arm/libsel4/cmake_install.cmake")
  include("/home/ethan/sel4test_ethan/build-arm/apps/sel4test-driver/util_libs/cmake_install.cmake")
  include("/home/ethan/sel4test_ethan/build-arm/apps/sel4test-driver/seL4_libs/cmake_install.cmake")
  include("/home/ethan/sel4test_ethan/build-arm/apps/sel4test-driver/sel4_projects_libs/cmake_install.cmake")
  include("/home/ethan/sel4test_ethan/build-arm/apps/sel4test-driver/libsel4testsupport/cmake_install.cmake")
  include("/home/ethan/sel4test_ethan/build-arm/apps/sel4test-driver/sel4test-tests/cmake_install.cmake")

endif()

