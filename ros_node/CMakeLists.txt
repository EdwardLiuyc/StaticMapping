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

file(GLOB srcs "*.cc")
add_executable(static_mapping_node static_mapping_node.cpp ${srcs})
target_link_libraries(static_mapping_node ${static_mapping_libs} ${require_libs})

# add_executable(join_maps_node ros_node/join_maps_node.cpp)
# target_link_libraries(join_maps_node ${static_mapping_libs} ${require_libs})