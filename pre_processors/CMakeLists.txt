file(GLOB srcs "*.cc")
add_library(${PROJECT_NAME}_pre_processors SHARED ${srcs})
target_link_libraries(${PROJECT_NAME}_pre_processors ${PROJECT_NAME}_common)

if(ENABLE_TEST)
  add_subdirectory(test)
endif(ENABLE_TEST)
