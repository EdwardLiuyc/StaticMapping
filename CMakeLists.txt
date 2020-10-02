cmake_minimum_required(VERSION 3.5.0)
project(static_mapping)

#######################################################
###  Compiler Options
#######################################################
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "-O3 -Wall \
  -Wextra -fPIC \
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
# option(USE_CUDA "Enable Cuda?" OFF)
# if(USE_CUDA)
#   find_package(CUDA REQUIRED)
#   include_directories(${CUDA_INCLUDE_DIRS})
  # find_package(CudaUtils REQUIRED)
  # include_directories(${CUDA_UTILS_INCLUDE_DIR})

  # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D_ICP_USE_CUDA_")
  # set_directory_properties(PROPERTIES COMPILE_DEFINITIONS "")

  # set(CUDA_CHECKER_TARGET_FILE ${PROJECT_SOURCE_DIR}/tools/check_cuda)
  # set(CUDA_CHECKER_SRC_FILE ${PROJECT_SOURCE_DIR}/tools/check_cuda.cu)
  # execute_process(COMMAND nvcc -lcuda ${CUDA_CHECKER_SRC_FILE} -o ${CUDA_CHECKER_TARGET_FILE})
  # execute_process(COMMAND ${CUDA_CHECKER_TARGET_FILE}
  #                 RESULT_VARIABLE CUDA_RETURN_CODE
  #                 OUTPUT_VARIABLE CUDA_ARCH)
  # if(${CUDA_RETURN_CODE} EQUAL 0)
  #   set(CUDA_SUCCESS TRUE)
  # else()
  #   set(CUDA_SUCCESS FALSE)
  # endif()

  # if (${CUDA_SUCCESS})
  #   message(STATUS "CUDA Architecture: ${CUDA_ARCH}")
  #   message(STATUS "CUDA Version: ${CUDA_VERSION_STRING}")
  #   message(STATUS "CUDA Path: ${CUDA_TOOLKIT_ROOT_DIR}")
  #   message(STATUS "CUDA Libararies: ${CUDA_LIBRARIES}")
  #   message(STATUS "CUDA Performance Primitives: ${CUDA_npp_LIBRARY}")
  # endif()
  # set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-O3;${CUDA_ARCH};--use_fast_math;-Xcompiler -fopenmp)

  # add_subdirectory(registrators/cuda)
  # list(APPEND require_libs registrators_cuda)

# endif(USE_CUDA)

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
# find_package(Ceres REQUIRED)
# include_directories(${CERES_INCLUDE_DIR})
# list(APPEND require_libs ceres)

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

find_package(TBB REQUIRED)
message(STATUS "TBB version: ${TBB_INTERFACE_VERSION}")
include_directories(${TBB_INCLUDE_DIRS})
list(APPEND require_libs ${TBB_LIBRARIES})

find_package(gflags REQUIRED)
list(APPEND require_libs ${GFLAGS_LIBRARIES})
find_package(Glog REQUIRED)
list(APPEND require_libs ${GLOG_LIBRARIES})

find_package(GeographicLib REQUIRED)
include_directories(${GeographicLib_INCLUDE_DIRS})
list(APPEND require_libs ${GeographicLib_LIBRARIES})

find_package(ImageMagick REQUIRED)
message(STATUS "Image Magick version : ${ImageMagick_VERSION_STRING}")
list(APPEND require_libs ${ImageMagick_LIBRARIES})

option(ENABLE_PROF "Enable profiling?" ON)
if(ENABLE_PROF)
  message(STATUS "Enable simple profiling.")
  add_definitions(-DENABLE_PROFILING)
  set(PROF_LOG_DIR ".static_mapping_log")
  add_definitions(-D_PROF_LOG_DIR_=".static_mapping_log")
  execute_process (COMMAND bash -c "mkdir -p ~/${PROF_LOG_DIR}")
endif(ENABLE_PROF)

option(ENABLE_TEST "" OFF)
if(ENABLE_TEST)
  find_package(Boost REQUIRED COMPONENTS unit_test_framework)
  enable_testing()
endif(ENABLE_TEST)

#######################################################
###  Source files
#######################################################
add_subdirectory(common)
add_subdirectory(descriptor)
add_subdirectory(back_end)
add_subdirectory(registrators)
add_subdirectory(builder)
add_subdirectory(third_parties)
#######################################################
###  Target
#######################################################
# link_libraries(glog)
list(APPEND static_mapping_libs ${PROJECT_NAME}_builder 
  ${PROJECT_NAME}_builder_data
  ${PROJECT_NAME}_backend 
  ${PROJECT_NAME}_registrators
  ${PROJECT_NAME}_descriptor
  ${PROJECT_NAME}_common
  pugixml)

message(STATUS "Local libs: ${static_mapping_libs}")
message(STATUS "Link libs: ${require_libs}")
message(STATUS "CXX FLAGS: ${CMAKE_CXX_FLAGS}")

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

#### ros cpp ####
add_subdirectory(ros_node)
