add_executable(test_bbox test_bbox.cc)
target_link_libraries(test_bbox gtest_main ${PROJECT_NAME}_common glog)
add_test(NAME test_bbox COMMAND test_bbox)

add_executable(test_octree test_octree.cc)
target_link_libraries(test_octree gtest_main ${PROJECT_NAME}_common glog)
add_test(NAME test_octree COMMAND test_octree)
