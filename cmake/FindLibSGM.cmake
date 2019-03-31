###############################################################################
# Find LibSGM
#
# This sets the following variables:
# LIBSGM_FOUND - True if LIBSGM was found.
# LIBSGM_INCLUDE_DIRS - Directories containing the LIBSGM include files.
# LIBSGM_LIBRARY - Libraries needed to use LIBSGM.

if(WIN32)
find_path(libSGM_DIR libSGM PATHS "C:/Program Files" "C:/Program Files (x86)")
set(libSGM_DIR ${libSGM_DIR}/libSGM)
else()
set(glog_DIR "")
endif()

# Find lib
set(LIBSGM_FOUND FALSE CACHE BOOL "" FORCE)
find_library(LIBSGM_LIBRARY
    NAMES sgm libsgm
    PATHS ${libSGM_DIR}
    PATH_SUFFIXES lib/
)

# Find include
find_path(LIBSGM_INCLUDE_DIRS
    NAMES libsgm.h
    PATHS ${libSGM_DIR}
    PATH_SUFFIXES include/
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(LibSGM DEFAULT_MSG LIBSGM_LIBRARY LIBSGM_INCLUDE_DIRS)

mark_as_advanced(LIBSGM_FOUND)

if(LIBSGM_FOUND)
	include_directories(${LIBSGM_INCLUDE_DIRS})
    set(LIBSGM_FOUND TRUE CACHE BOOL "" FORCE)
    set(LIBSGM_LIBRARIES ${LIBSGM_LIBRARY})
    message(STATUS "Found libSGM")
endif()
