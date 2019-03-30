###############################################################################
# Find glog
#

if(WIN32)
find_path(glog_DIR glog PATHS "C:/Program Files" "C:/Program Files (x86)")
set(glog_DIR ${glog_DIR}/glog)
else()
set(glog_DIR "")
endif()

# Find lib
set(GLOG_FOUND FALSE CACHE BOOL "" FORCE)
find_library(GLOG_LIBRARY
    NAMES glog libglog
    PATHS ${glog_DIR}
    PATH_SUFFIXES lib/
)

# Find include
find_path(GLOG_INCLUDE_DIRS
    NAMES glog/logging.h
    PATHS ${glog_DIR}
    PATH_SUFFIXES include
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(glog DEFAULT_MSG GLOG_LIBRARY GLOG_INCLUDE_DIRS)

message(STATUS "(GLOG_FOUND : ${GLOG_FOUND} include: ${GLOG_INCLUDE_DIRS}, lib: ${GLOG_LIBRARY})")

mark_as_advanced(GLOG_FOUND)

if(GLOG_FOUND)
	include_directories(${GLOG_INCLUDE_DIRS})
    set(GLOG_FOUND TRUE CACHE BOOL "" FORCE)
    set(GLOG_LIBRARIES ${GLOG_LIBRARY})
    message(STATUS "glog found ( include: ${GLOG_INCLUDE_DIRS}, lib: ${GLOG_LIBRARY})")
endif()
