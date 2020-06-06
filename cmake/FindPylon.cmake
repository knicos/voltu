###############################################################################
# Find Pylon
#

set(PYLON_FOUND FALSE CACHE BOOL "" FORCE)

if(WIN32)
find_path(PYLON_DIR NAMES include/pylon/PylonBase.h PATHS "C:/Program Files/Pylon" "C:/Program Files (x86)/Pylon")
else()
find_path(PYLON_DIR NAMES include/pylon/PylonBase.h PATHS "/opt/pylon" "/opt/pylon6")
endif()

if (PYLON_DIR)
	set(PYLON_FOUND TRUE CACHE BOOL "" FORCE)
	set(HAVE_PYLON TRUE)
	# Find lib dir
	
	# Find include
	find_path(PYLON_LIBRARY_DIRS
		NAMES libpylonbase.so
		PATHS ${PYLON_DIR}
		PATH_SUFFIXES lib
	)

	# Find include
	find_path(PYLON_INCLUDE_DIRS
		NAMES pylon/PylonBase.h
		PATHS ${PYLON_DIR}
		PATH_SUFFIXES include
	)

	include(FindPackageHandleStandardArgs)
	find_package_handle_standard_args(Pylon DEFAULT_MSG PYLON_DIR)

	mark_as_advanced(PYLON_FOUND)
	mark_as_advanced(PYLON_INCLUDE_DIRS)
	mark_as_advanced(PYLON_LIBRARY_DIRS)

	list(APPEND PYLON_LIBRARIES pylonbase pylonutility GenApi_gcc_v3_1_Basler_pylon GCBase_gcc_v3_1_Basler_pylon)

	add_library(Pylon UNKNOWN IMPORTED)
	set_property(TARGET Pylon PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${PYLON_INCLUDE_DIRS})
	set_property(TARGET Pylon PROPERTY INTERFACE_LINK_DIRECTORIES ${PYLON_INCLUDE_DIRS})
	set_property(TARGET Pylon PROPERTY INTERFACE_LINK_LIBRARIES ${PYLON_LIBRARIES})
else()
	add_library(Pylon INTERFACE)
endif()
