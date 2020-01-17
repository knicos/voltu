set(STREAMSRC
	src/stream.cpp
	src/filestream.cpp
	src/receiver.cpp
	src/sender.cpp
	src/netstream.cpp
	src/injectors.cpp
	src/parsers.cpp
)

add_library(ftlstreams ${STREAMSRC})

# target_compile_options(ftlrgbd PUBLIC $<$<COMPILE_LANGUAGE:CXX>:-fPIC>)
# target_compile_options(ftlrgbd PUBLIC "-DMAKE_SHARED")

target_include_directories(ftlstreams PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)

#target_include_directories(cv-node PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(ftlstreams ftlrgbd ftlcommon ${OpenCV_LIBS} Eigen3::Eigen ftlnet ftlcodecs)

add_subdirectory(test)