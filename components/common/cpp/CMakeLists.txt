set(COMMONSRC
	src/config.cpp
	src/uri.cpp
	src/configuration.cpp
	src/configurable.cpp
	src/loguru.cpp
	src/cuda_common.cpp
	src/ctpl_stl.cpp
	src/timer.cpp
)

check_function_exists(uriParseSingleUriA HAVE_URIPARSESINGLE)

add_library(ftlcommon ${COMMONSRC})

target_compile_options(ftlcommon PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-fPIC>)

target_include_directories(ftlcommon PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)
target_link_libraries(ftlcommon Threads::Threads ${OS_LIBS} ${OpenCV_LIBS} ${URIPARSER_LIBRARIES} ${CUDA_LIBRARIES})

add_subdirectory(test)

