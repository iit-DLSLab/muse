# Install script for directory: /home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/home/ynistico-iit.local/miniforge3/envs/muse/bin/x86_64-conda-linux-gnu-objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/install" TYPE PROGRAM FILES "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/build/series_elastic_actuator_msgs/catkin_generated/installspace/_setup_util.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/install" TYPE PROGRAM FILES "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/build/series_elastic_actuator_msgs/catkin_generated/installspace/env.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/install/setup.bash;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/install/local_setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/install" TYPE FILE FILES
    "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/build/series_elastic_actuator_msgs/catkin_generated/installspace/setup.bash"
    "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/build/series_elastic_actuator_msgs/catkin_generated/installspace/local_setup.bash"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/install/setup.sh;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/install/local_setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/install" TYPE FILE FILES
    "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/build/series_elastic_actuator_msgs/catkin_generated/installspace/setup.sh"
    "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/build/series_elastic_actuator_msgs/catkin_generated/installspace/local_setup.sh"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/install/setup.zsh;/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/install/local_setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/install" TYPE FILE FILES
    "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/build/series_elastic_actuator_msgs/catkin_generated/installspace/setup.zsh"
    "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/build/series_elastic_actuator_msgs/catkin_generated/installspace/local_setup.zsh"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/install" TYPE FILE FILES "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/build/series_elastic_actuator_msgs/catkin_generated/installspace/.rosinstall")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/series_elastic_actuator_msgs/msg" TYPE FILE FILES
    "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommand.msg"
    "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorCommands.msg"
    "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReading.msg"
    "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadingExtended.msg"
    "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadings.msg"
    "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorReadingsExtended.msg"
    "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorState.msg"
    "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorStateExtended.msg"
    "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/msg/SeActuatorStates.msg"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/series_elastic_actuator_msgs/cmake" TYPE FILE FILES "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/build/series_elastic_actuator_msgs/catkin_generated/installspace/series_elastic_actuator_msgs-msg-paths.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/devel/.private/series_elastic_actuator_msgs/include/series_elastic_actuator_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/devel/.private/series_elastic_actuator_msgs/share/roseus/ros/series_elastic_actuator_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/devel/.private/series_elastic_actuator_msgs/share/common-lisp/ros/series_elastic_actuator_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/devel/.private/series_elastic_actuator_msgs/share/gennodejs/ros/series_elastic_actuator_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/home/ynistico-iit.local/miniforge3/envs/muse/bin/python3.11" -m compileall "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/devel/.private/series_elastic_actuator_msgs/lib/python3.11/site-packages/series_elastic_actuator_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.11/site-packages" TYPE DIRECTORY FILES "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/devel/.private/series_elastic_actuator_msgs/lib/python3.11/site-packages/series_elastic_actuator_msgs")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/build/series_elastic_actuator_msgs/catkin_generated/installspace/series_elastic_actuator_msgs.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/series_elastic_actuator_msgs/cmake" TYPE FILE FILES "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/build/series_elastic_actuator_msgs/catkin_generated/installspace/series_elastic_actuator_msgs-msg-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/series_elastic_actuator_msgs/cmake" TYPE FILE FILES
    "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/build/series_elastic_actuator_msgs/catkin_generated/installspace/series_elastic_actuator_msgsConfig.cmake"
    "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/build/series_elastic_actuator_msgs/catkin_generated/installspace/series_elastic_actuator_msgsConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/series_elastic_actuator_msgs" TYPE FILE FILES "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/src/series_elastic_actuator_msgs/package.xml")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/build/series_elastic_actuator_msgs/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
if(CMAKE_INSTALL_COMPONENT)
  if(CMAKE_INSTALL_COMPONENT MATCHES "^[a-zA-Z0-9_.+-]+$")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
  else()
    string(MD5 CMAKE_INST_COMP_HASH "${CMAKE_INSTALL_COMPONENT}")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INST_COMP_HASH}.txt")
    unset(CMAKE_INST_COMP_HASH)
  endif()
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/home/ynistico-iit.local/dls_ws_home/muse/muse_ws/build/series_elastic_actuator_msgs/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
