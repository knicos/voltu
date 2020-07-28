#add_library(FtlStream OBJECT src/stream.cpp)
#target_include_directories(FtlStream PUBLIC
#	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#	$<INSTALL_INTERFACE:include>
#	PRIVATE src)
#add_dependencies(FtlStream ftlcommon)

set(STREAMSRC
	src/stream.cpp
	src/filestream.cpp
	src/receiver.cpp
	src/sender.cpp
	src/feed.cpp
	src/netstream.cpp
	src/injectors.cpp
	src/parsers.cpp
	src/builder.cpp
	src/renderer.cpp
	src/renderers/screen_render.cpp
	src/renderers/openvr_render.cpp
	src/renderers/collisions.cpp
)

add_library(ftlstreams ${STREAMSRC})

# target_compile_options(ftlrgbd PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-fPIC>)
# target_compile_options(ftlrgbd PUBLIC "-DMAKE_SHARED")

target_include_directories(ftlstreams PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)

#target_include_directories(cv-node PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(ftlstreams ftlrgbd ftlrender ftlcommon ${OpenCV_LIBS} Eigen3::Eigen ftlnet ftlcodecs ftlaudio openvr)

target_precompile_headers(ftlstreams REUSE_FROM ftldata)

if (BUILD_TESTS)
add_subdirectory(test)
endif()
