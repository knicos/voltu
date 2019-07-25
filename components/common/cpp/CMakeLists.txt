set(COMMONSRC
	src/config.cpp
	src/uri.cpp
	src/configuration.cpp
	src/configurable.cpp
	src/loguru.cpp
	src/opencv_to_pcl.cpp
	src/cuda_common.cpp
)

check_function_exists(uriParseSingleUriA HAVE_URIPARSESINGLE)

add_library(ftlcommon ${COMMONSRC})

target_compile_options(ftlcommon PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-fPIC>)

target_include_directories(ftlcommon PUBLIC
	${PCL_INCLUDE_DIRS}
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)
target_link_libraries(ftlcommon Threads::Threads ${OS_LIBS} ${OpenCV_LIBS} ${PCL_LIBRARIES} ${URIPARSER_LIBRARIES})

add_subdirectory(test)

