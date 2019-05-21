set(COMMONSRC
	src/config.cpp
	src/configuration.cpp
	src/opencv_to_pcl.cpp
)

add_library(ftlcommon ${COMMONSRC})

target_include_directories(ftlcommon PUBLIC
	${PCL_INCLUDE_DIRS}
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)
target_link_libraries(ftlcommon glog::glog ${OpenCV_LIBS} ${PCL_LIBRARIES})

