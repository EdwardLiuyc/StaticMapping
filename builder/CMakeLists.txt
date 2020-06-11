file(GLOB srcs "*.cc")
# TODO(edward) currently no use
# file(GLOB sensors_fusion_srcs "sensor_fusions/*.cc")
# add_library(${PROJECT_NAME}_builder SHARED ${srcs} ${sensors_fusion_srcs})
add_library(${PROJECT_NAME}_builder SHARED ${srcs})