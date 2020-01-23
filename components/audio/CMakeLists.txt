set(AUDIOSRC
	src/source.cpp
	src/frame.cpp
	src/portaudio.cpp
	src/speaker.cpp
)

add_library(ftlaudio ${AUDIOSRC})

target_include_directories(ftlaudio PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)

target_link_libraries(ftlaudio ftlcommon Eigen3::Eigen ftlstreams ftldata portaudio)

#add_subdirectory(test)

