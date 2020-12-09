function(add_boost_test test_name)
  add_executable(${test_name} ${test_name}.cc)
  target_include_directories(${test_name} PRIVATE ${Boost_INCLUDE_DIRS})
  target_compile_definitions(${test_name} PRIVATE "BOOST_TEST_DYN_LINK=1")
  target_link_libraries(${test_name} 
    ${PROJECT_NAME}_builder_data 
    ${PROJECT_NAME}_common
    ${PROJECT_NAME}_test_helper
    ${PROJECT_NAME}_pre_processors
    ${GeographicLib_LIBRARIES} 
    glog
    pugixml)
  target_link_libraries(${test_name} ${Boost_UNIT_TEST_FRAMEWORK_LIBRARY})
  add_test(NAME ${test_name} COMMAND ${test_name})
endfunction()

add_boost_test(test_filter_axis_range)
add_boost_test(test_filter_bounding_box)
add_boost_test(test_filter_range)
add_boost_test(test_filter_voxel_grid)
add_boost_test(test_filter_random_sample)
