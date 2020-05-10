file(GLOB srcs "*.cc")
add_library(${PROJECT_NAME}_common STATIC ${srcs} ./performance/simple_prof.cc)