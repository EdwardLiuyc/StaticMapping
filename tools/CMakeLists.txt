add_executable(pcd_sampler pcd_sampler.cc)
add_executable(path_statistic path_statistic.cc)
add_executable(join_pieces join_pieces.cc)
target_link_libraries(join_pieces pugixml)
add_executable(pcl_viewer_normals pcl_viewer_normals.cc)

# no need to compress the pointcloud
# even there is a need for compression, use zlib instead
# find_package(Blosc)
# if(BLOSC_FOUND)
#   message(STATUS "Found Blosc, build the compressor tool.")
#   include_directories(${BLOSC_INCLUDE_DIR})
#   link_directories(${BLOSC_LIBRARYDIR})
#   add_executable(cloud_compressor cloud_compressor.cc)
#   target_link_libraries(cloud_compressor blosc pthread)
# endif()

find_package(LibLAS)
if(LIBLAS_FOUND)
  include_directories(${LIBLAS_INCLUDE_DIR})
  add_executable(pcd_to_las pcd_to_las.cc)
  target_link_libraries(pcd_to_las las)
endif(LIBLAS_FOUND)