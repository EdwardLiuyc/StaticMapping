# Install script for directory: /home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab

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
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/matlab/geographiclib" TYPE FILE FILES
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/Contents.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/cassini_fwd.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/cassini_inv.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/defaultellipsoid.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/ecc2flat.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/eqdazim_fwd.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/eqdazim_inv.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/flat2ecc.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/gedistance.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/gedoc.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/geocent_fwd.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/geocent_inv.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/geodarea.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/geoddistance.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/geoddoc.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/geodreckon.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/geographiclib_test.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/geoid_height.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/geoid_load.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/gereckon.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/gnomonic_fwd.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/gnomonic_inv.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/loccart_fwd.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/loccart_inv.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/mgrs_fwd.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/mgrs_inv.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/polarst_fwd.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/polarst_inv.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/projdoc.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/tranmerc_fwd.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/tranmerc_inv.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/utmups_fwd.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/utmups_inv.m"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/matlab/geographiclib/private" TYPE FILE FILES
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/private/A1m1f.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/private/A2m1f.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/private/A3coeff.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/private/A3f.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/private/AngDiff.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/private/AngNormalize.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/private/AngRound.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/private/C1f.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/private/C1pf.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/private/C2f.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/private/C3coeff.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/private/C3f.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/private/C4coeff.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/private/C4f.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/private/G4coeff.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/private/GeoRotation.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/private/LatFix.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/private/SinCosSeries.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/private/atan2dx.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/private/cbrtx.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/private/copysignx.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/private/cvmgt.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/private/eatanhe.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/private/geoid_file.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/private/geoid_load_file.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/private/norm2.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/private/remx.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/private/sincosdx.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/private/sumx.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/private/swap.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/private/tauf.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib/private/taupf.m"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/matlab/geographiclib-legacy" TYPE FILE FILES
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib-legacy/Contents.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib-legacy/geocentricforward.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib-legacy/geocentricreverse.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib-legacy/geodesicdirect.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib-legacy/geodesicinverse.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib-legacy/geodesicline.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib-legacy/geoidheight.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib-legacy/localcartesianforward.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib-legacy/localcartesianreverse.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib-legacy/mgrsforward.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib-legacy/mgrsreverse.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib-legacy/polygonarea.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib-legacy/utmupsforward.m"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/matlab/geographiclib-legacy/utmupsreverse.m"
    )
endif()

