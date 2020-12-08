file(GLOB srcs "*.cc")
add_library(${PROJECT_NAME}_pre_processors SHARED ${srcs})

if(ENABLE_TEST)
  add_subdirectory(test)
endif(ENABLE_TEST)
