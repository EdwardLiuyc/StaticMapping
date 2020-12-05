file(GLOB srcs "*.cc")
add_library(${PROJECT_NAME}_builder_data SHARED ${srcs})

if(ENABLE_TEST)
  add_subdirectory(test)
endif(ENABLE_TEST)