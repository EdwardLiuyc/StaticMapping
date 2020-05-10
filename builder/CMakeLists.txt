file(GLOB srcs "*.cc")
file(GLOB sensors_fusion_srcs "sensor_fusions/*.cc")
add_library(${PROJECT_NAME}_builder STATIC ${srcs} ${sensors_fusion_srcs})