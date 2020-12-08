file(GLOB srcs "*.cc")
add_library(${PROJECT_NAME}_test_helper SHARED ${srcs})
