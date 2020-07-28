add_library(Loguru OBJECT src/loguru.cpp)
target_include_directories(Loguru PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

set(COMMONSRC
	src/config.cpp
	src/uri.cpp
	src/configuration.cpp
	src/configurable.cpp
	$<TARGET_OBJECTS:Loguru>
	src/cuda_common.cpp
	src/ctpl_stl.cpp
	src/timer.cpp
	src/profiler.cpp
	src/exception.cpp
	src/utility/base64.cpp
)

check_function_exists(uriParseSingleUriA HAVE_URIPARSESINGLE)

add_library(ftlcommon ${COMMONSRC})

if (NOT WIN32)
target_compile_options(ftlcommon PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-fPIC>)
endif()

target_include_directories(ftlcommon PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)
target_link_libraries(ftlcommon Threads::Threads Eigen3::Eigen ${OS_LIBS} ${OpenCV_LIBS} ${URIPARSER_LIBRARIES} ${CUDA_LIBRARIES})

target_precompile_headers(ftlcommon
	PRIVATE include/ftl/cuda_common.hpp
	PRIVATE include/loguru.hpp
)

set_property(TARGET ftlcommon PROPERTY CUDA_ARCHITECTURES OFF)

if (BUILD_TESTS)
add_subdirectory(test)
endif()

