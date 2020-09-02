# Install script for directory: /home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib

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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/GeographicLib" TYPE FILE FILES
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/Accumulator.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/AlbersEqualArea.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/AzimuthalEquidistant.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/CassiniSoldner.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/CircularEngine.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/Constants.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/DMS.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/Ellipsoid.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/EllipticFunction.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/GARS.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/GeoCoords.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/Geocentric.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/Geodesic.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/GeodesicExact.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/GeodesicLine.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/GeodesicLineExact.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/Geohash.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/Geoid.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/Georef.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/Gnomonic.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/GravityCircle.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/GravityModel.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/LambertConformalConic.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/LocalCartesian.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/MGRS.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/MagneticCircle.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/MagneticModel.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/Math.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/NearestNeighbor.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/NormalGravity.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/OSGB.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/PolarStereographic.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/PolygonArea.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/Rhumb.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/SphericalEngine.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/SphericalHarmonic.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/SphericalHarmonic1.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/SphericalHarmonic2.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/TransverseMercator.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/TransverseMercatorExact.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/UTMUPS.hpp"
    "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/include/GeographicLib/Utility.hpp"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/GeographicLib" TYPE FILE FILES "/home/robotics/Lab_Project/ROS_WorkSpace/StaticMapping/setup/GeographicLib-1.50.1/build/include/GeographicLib/Config.h")
endif()

