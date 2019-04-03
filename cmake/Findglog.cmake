###############################################################################
# Find glog
#

if(WIN32)
find_path(glog_DIR NAMES include/glog/logging.h PATHS "C:/Program Files/glog" "C:/Program Files/google-glog" "C:/Program Files (x86)/google-glog")
set(glog_DIR ${glog_DIR})
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

mark_as_advanced(GLOG_FOUND)

if(GLOG_FOUND)
    set(GLOG_FOUND TRUE CACHE BOOL "" FORCE)
    mark_as_advanced(GLOG_INCLUDE_DIRS GLOG_LIBRARY)
    add_library(glog::glog UNKNOWN IMPORTED)
    set_property(TARGET glog::glog PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${GLOG_INCLUDE_DIRS})
	set_property(TARGET glog::glog PROPERTY IMPORTED_LOCATION ${GLOG_LIBRARY})
    message(STATUS "Found glog: ${GLOG_LIBRARY}")
endif()
