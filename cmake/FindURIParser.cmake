###############################################################################
# Find URI Parser
#

if(WIN32)
find_path(URIP_DIR NAMES include/uriparser/Uri.h PATHS "C:/Program Files/uriparser" "C:/Program Files (x86)/uriparser")
else()
set(URIP_DIR "")
endif()

# Find lib
set(URIPARSER_FOUND FALSE CACHE BOOL "" FORCE)
find_library(URIPARSER_LIBRARY
    NAMES uriparser liburiparser
    PATHS ${URIP_DIR}
    PATH_SUFFIXES lib/
)

# Find include
find_path(URIPARSER_INCLUDE_DIRS
    NAMES uriparser/Uri.h
    PATHS ${URIP_DIR}
    PATH_SUFFIXES include
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(URIParser DEFAULT_MSG URIPARSER_LIBRARY URIPARSER_INCLUDE_DIRS)

mark_as_advanced(URIPARSER_FOUND)

if(URIPARSER_FOUND)
	include_directories(${URIPARSER_INCLUDE_DIRS})
    set(URIPARSER_FOUND TRUE CACHE BOOL "" FORCE)
    set(URIPARSER_LIBRARIES "${URIPARSER_LIBRARY}")
    message(STATUS "Found URIParser")
endif()
