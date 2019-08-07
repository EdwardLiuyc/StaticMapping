add_executable(pcd_sampler pcd_sampler.cc)
add_executable(fuse_with_hdmap fuse_with_hdmap.cc)
add_executable(path_statistic path_statistic.cc)
add_executable(join_pieces join_pieces.cc ../common/pugixml.cc)

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
