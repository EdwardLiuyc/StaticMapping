file(GLOB srcs "*.cc")
file(GLOB perf_srcs "performance/*.cc")
add_library(${PROJECT_NAME}_common SHARED ${srcs} ${perf_srcs})

if(ENABLE_TEST)
  add_subdirectory(test)
endif(ENABLE_TEST)

if(ENABLE_EXAMPLE)
  add_subdirectory(example)
endif(ENABLE_EXAMPLE)