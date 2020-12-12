file(GLOB srcs "*.cc")
add_library(${PROJECT_NAME}_builder_data SHARED ${srcs})
target_link_libraries(${PROJECT_NAME}_builder_data ${GeographicLib_LIBRARIES})

if(ENABLE_TEST)
  add_subdirectory(test)
endif(ENABLE_TEST)