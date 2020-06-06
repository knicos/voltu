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
	
	include(FindPackageHandleStandardArgs)
	find_package_handle_standard_args(Pylon DEFAULT_MSG PYLON_DIR)

	mark_as_advanced(PYLON_FOUND)

	list(APPEND PYLON_LIBRARIES pylonbase pylonutility GenApi_gcc_v3_1_Basler_pylon GCBase_gcc_v3_1_Basler_pylon)

	add_library(Pylon INTERFACE)
	set_property(TARGET Pylon PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${PYLON_DIR}/include)
	#set_property(TARGET Pylon PROPERTY INTERFACE_LINK_DIRECTORIES ${PYLON_DIR}/lib)
	link_directories(${PYLON_DIR}/lib)
	set_property(TARGET Pylon PROPERTY INTERFACE_LINK_LIBRARIES ${PYLON_LIBRARIES})
else()
	add_library(Pylon INTERFACE)
endif()
