add_library(ftlrender
	src/display.cpp
)

# These cause errors in CI build and are being removed from PCL in newer versions
# target_compile_options(ftlrender PUBLIC ${PCL_DEFINITIONS})

target_include_directories(ftlrender PUBLIC
	${PCL_INCLUDE_DIRS}
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)
target_link_libraries(ftlrender Threads::Threads glog::glog ${OpenCV_LIBS} ${PCL_LIBRARIES})

#ADD_SUBDIRECTORY(test)
