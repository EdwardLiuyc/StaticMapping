file(GLOB srcs "*.cc")
add_library(${PROJECT_NAME}_pre_processors SHARED ${srcs})