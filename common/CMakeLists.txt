file(GLOB srcs "*.cc")
file(GLOB perf_srcs "performance/*.cc")
add_library(${PROJECT_NAME}_common SHARED ${srcs} ${perf_srcs})