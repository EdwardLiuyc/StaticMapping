cmake_minimum_required(VERSION 3.5.0)
project(static_mapping)
set(TARGET_LIB_NAME static_mapping)

#######################################################
###  Compiler Options
#######################################################
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-O3 -Wall \
  -Wextra -fPIC -ffast-math \
  -Wno-sign-compare \
  -Wno-unused-parameter \
  -Wno-unused-but-set-variable ")
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

### enable openmp or not ###
option(USE_OPENMP "Enable OpenMP?" ON) # set to OFF to disable
if(USE_OPENMP)
 FIND_PACKAGE(OpenMP)
 if(OPENMP_FOUND OR OpenMP_FOUND)
  message(STATUS "OpenMP flags = ${OpenMP_CXX_FLAGS}")
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
 else(OPENMP_FOUND OR OpenMP_FOUND)
  message(STATUS "OpenMP not supported")
 endif(OPENMP_FOUND OR OpenMP_FOUND)
endif(USE_OPENMP)

#######################################################
###  Dependencies
#######################################################
include_directories("third_parties" "." "builder")
set(require_libs )

#### cuda ####
option(USE_CUDA "Enable Cuda?" OFF)
if(USE_CUDA)
  find_package(CUDA REQUIRED)
  include_directories(${CUDA_INCLUDE_DIRS})
  find_package(CudaUtils REQUIRED)
  include_directories(${CUDA_UTILS_INCLUDE_DIR})

  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_ICP_USE_CUDA_")
  set_directory_properties(PROPERTIES COMPILE_DEFINITIONS "")

  set(CUDA_CHECKER_TARGET_FILE ${PROJECT_SOURCE_DIR}/tools/check_cuda)
  set(CUDA_CHECKER_SRC_FILE ${PROJECT_SOURCE_DIR}/tools/check_cuda.cu)
  execute_process(COMMAND nvcc -lcuda ${CUDA_CHECKER_SRC_FILE} -o ${CUDA_CHECKER_TARGET_FILE})
  execute_process(COMMAND ${CUDA_CHECKER_TARGET_FILE}
                  RESULT_VARIABLE CUDA_RETURN_CODE
                  OUTPUT_VARIABLE CUDA_ARCH)
  if(${CUDA_RETURN_CODE} EQUAL 0)
    set(CUDA_SUCCESS TRUE)
  else()
    set(CUDA_SUCCESS FALSE)
  endif()

  if (${CUDA_SUCCESS})
    message(STATUS "CUDA Architecture: ${CUDA_ARCH}")
    message(STATUS "CUDA Version: ${CUDA_VERSION_STRING}")
    message(STATUS "CUDA Path: ${CUDA_TOOLKIT_ROOT_DIR}")
    message(STATUS "CUDA Libararies: ${CUDA_LIBRARIES}")
    message(STATUS "CUDA Performance Primitives: ${CUDA_npp_LIBRARY}")
  endif()
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-O3;${CUDA_ARCH};--use_fast_math;-Xcompiler -fopenmp)

  add_subdirectory(registrators/cuda)
  list(APPEND require_libs registrators_cuda)

endif(USE_CUDA)

add_subdirectory(third_parties/libicp)
list(APPEND require_libs icp)

#### pcl ####
find_package(PCL REQUIRED QUIET)
if(${PCL_VERSION} VERSION_GREATER "1.8.0" OR 
   ${PCL_VERSION} VERSION_EQUAL "1.8.0")
  message(STATUS "pcl version ${PCL_VERSION}")
  ## it is a trick to remove annoying warning in pcl-1.8 like this
  ## warning: ‘pcl::SAC_SAMPLE_SIZE’ ... [-Wdeprecated-declarations]
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -isystem /usr/include/pcl-1.8") 
endif()
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
link_libraries(${PCL_LIBRARIES})
#### ros cpp ####
find_package(catkin COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
  tf
  tf_conversions
  pcl_conversions
  urdf
  rosbag
  REQUIRED
)
include_directories(${catkin_INCLUDE_DIRS})
link_libraries(${catkin_LIBRARIES})
add_subdirectory(tools)
#### gtsam ####
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})
list(APPEND require_libs gtsam)
#### png #### 
find_package(PNG REQUIRED)
include_directories(${PNG_INCLUDE_DIR})
message( STATUS "Found PNG in dir: ${PNG_INCLUDE_DIR}")
list(APPEND require_libs ${PNG_LIBRARY})
#### eigen ####
find_package(Eigen3 3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
message( STATUS 
  "Found Eigen in dir: ${EIGEN3_INCLUDE_DIR} version: ${EIGEN3_VERSION}")
#### libpointmatcher ####
find_package(libpointmatcher REQUIRED)
include_directories(${libpointmatcher_INCLUDE_DIRS})
list(APPEND require_libs ${libpointmatcher_LIBRARIES})
#### ceres ####
find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIR})
list(APPEND require_libs ceres)

# option(USE_OPENVDB "Use Openvdb?" OFF)
# if(USE_OPENVDB)
#   find_package(OpenVDB)
#   if(OpenVDB_FOUND)
#     message(STATUS "OpenVDB Found.")
#     include_directories(${OpenVDB_INCLUDE_DIR})
#     link_directories(${OpenVDB_LIBRARY_DIR})
#     link_libraries(${OpenVDB_LIBRARIES})
#     add_definitions(-D_USE_OPENVDB_)
#   endif(OpenVDB_FOUND)
# endif(USE_OPENVDB)

option(USE_TBB "Enable TBB?" ON) #set to OFF to disable
if(USE_TBB)
  find_package(TBB)
  if(TBB_FOUND)
    message(STATUS "TBB version: ${TBB_INTERFACE_VERSION}")
    include_directories(${TBB_INCLUDE_DIRS})
    list(APPEND require_libs ${TBB_LIBRARIES})
    add_definitions(-D_USE_TBB_)
  endif(TBB_FOUND)
endif(USE_TBB)

find_package(gflags REQUIRED)
list(APPEND require_libs ${GFLAGS_LIBRARIES})

option(ENBALE_PROF "Enable profiling?" OFF)
if(ENBALE_PROF)
  add_definitions(-DENBALE_PROFILING)
  set(PROF_LOG_DIR ".static_mapping_log")
  add_definitions(-D_PROF_LOG_DIR_=".static_mapping_log")
  execute_process (COMMAND bash -c "mkdir -p ~/${PROF_LOG_DIR}")
endif(ENBALE_PROF)

#######################################################
###  Source files
#######################################################
add_subdirectory(common)
add_subdirectory(descriptor)
add_subdirectory(back_end)

file(GLOB srcs "builder/*.cc" ) 
file(GLOB sensors_fusion_srcs "builder/sensor_fusions/*.cc")
file(GLOB registrators_srcs "registrators/*.cc")
#######################################################
###  Target
#######################################################
# link_libraries(glog)
list(APPEND require_libs ${PROJECT_NAME}_common
  ${PROJECT_NAME}_descriptor
  ${PROJECT_NAME}_backend)

message(STATUS "Link libs: ${require_libs}")
message(STATUS "CXX FLAGS: ${CMAKE_CXX_FLAGS}")

add_library(${TARGET_LIB_NAME} 
  ${srcs} ${sensors_fusion_srcs} 
  ${registrators_srcs})
# target_link_libraries(${TARGET_LIB_NAME} ${OpenCV_LIBRARIES})

if($ENV{ROS_DISTRO} STREQUAL "melodic")
  message(STATUS "Using ROS melodic")
  add_definitions(-DURDFDOM_HEADERS_HAS_SHARED_PTR_DEFS=1)
  # Do stuff specific to Kinetic
elseif($ENV{ROS_DISTRO} STREQUAL "kinetic")
  message(STATUS "Using ROS kinetic")
else()
  message(STATUS "Unknown ROS distro:")
  message(STATUS $ENV{ROS_DISTRO})
endif()

add_executable(static_mapping_node 
  ros_node/static_mapping_node.cpp
  ros_node/urdf_reader.cc
  ros_node/tf_bridge.cc
  ros_node/playable_bag.cc
  ros_node/kitti_reader.cc)
target_link_libraries(static_mapping_node ${TARGET_LIB_NAME} ${require_libs})

add_executable(join_maps_node ros_node/join_maps_node.cpp)
target_link_libraries(join_maps_node ${TARGET_LIB_NAME} ${require_libs})
