# Install script for directory: /home/weiggen/wei_research_ws/src/voronoi_cbsa

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/weiggen/wei_research_ws/install")
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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/voronoi_cbsa/msg" TYPE FILE FILES
    "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ExchangeData.msg"
    "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/VoteList.msg"
    "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/NeighborInfo.msg"
    "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/NeighborInfoArray.msg"
    "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ExchangeDataArray.msg"
    "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/TargetInfo.msg"
    "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/TargetInfoArray.msg"
    "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/SensorArray.msg"
    "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Sensor.msg"
    "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/ValidSensors.msg"
    "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/Weight.msg"
    "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/WeightArray.msg"
    "/home/weiggen/wei_research_ws/src/voronoi_cbsa/msg/densityGradient.msg"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/voronoi_cbsa/cmake" TYPE FILE FILES "/home/weiggen/wei_research_ws/build/voronoi_cbsa/catkin_generated/installspace/voronoi_cbsa-msg-paths.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/weiggen/wei_research_ws/devel/include/voronoi_cbsa")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/roseus/ros" TYPE DIRECTORY FILES "/home/weiggen/wei_research_ws/devel/share/roseus/ros/voronoi_cbsa")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/weiggen/wei_research_ws/devel/share/common-lisp/ros/voronoi_cbsa")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/gennodejs/ros" TYPE DIRECTORY FILES "/home/weiggen/wei_research_ws/devel/share/gennodejs/ros/voronoi_cbsa")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "/usr/bin/python3" -m compileall "/home/weiggen/wei_research_ws/devel/lib/python3/dist-packages/voronoi_cbsa")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3/dist-packages" TYPE DIRECTORY FILES "/home/weiggen/wei_research_ws/devel/lib/python3/dist-packages/voronoi_cbsa")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/weiggen/wei_research_ws/build/voronoi_cbsa/catkin_generated/installspace/voronoi_cbsa.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/voronoi_cbsa/cmake" TYPE FILE FILES "/home/weiggen/wei_research_ws/build/voronoi_cbsa/catkin_generated/installspace/voronoi_cbsa-msg-extras.cmake")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/voronoi_cbsa/cmake" TYPE FILE FILES
    "/home/weiggen/wei_research_ws/build/voronoi_cbsa/catkin_generated/installspace/voronoi_cbsaConfig.cmake"
    "/home/weiggen/wei_research_ws/build/voronoi_cbsa/catkin_generated/installspace/voronoi_cbsaConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/voronoi_cbsa" TYPE FILE FILES "/home/weiggen/wei_research_ws/src/voronoi_cbsa/package.xml")
endif()

