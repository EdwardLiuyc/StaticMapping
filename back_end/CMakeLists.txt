file(GLOB srcs "*.cc")
add_library(${PROJECT_NAME}_backend SHARED ${srcs})