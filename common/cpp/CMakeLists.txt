set(COMMONSRC
	src/config.cpp
	src/configuration.cpp
)

add_library(ftlcommon ${COMMONSRC})

target_include_directories(ftlcommon PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)
target_link_libraries(ftlcommon glog::glog)

