# directories
set (LIBICP_SRC_DIR src)
# include directory
include_directories("${LIBICP_SRC_DIR}")
# sources
FILE(GLOB LIBICP_SRC_FILES "*.cpp")

add_library(icp STATIC ${LIBICP_SRC_FILES})