set(CODECSRC
	src/bitrates.cpp
	src/encoder.cpp
	src/decoder.cpp
	src/opencv_encoder.cpp
	src/opencv_decoder.cpp
	src/generate.cpp
	src/writer.cpp
	src/reader.cpp
	src/channels.cpp
)

if (HAVE_NVPIPE)
	list(APPEND CODECSRC src/nvpipe_encoder.cpp)
	list(APPEND CODECSRC src/nvpipe_decoder.cpp)
endif()

add_library(ftlcodecs ${CODECSRC})

target_include_directories(ftlcodecs PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)

#target_include_directories(cv-node PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(ftlcodecs ftlcommon ${OpenCV_LIBS} ${CUDA_LIBRARIES} Eigen3::Eigen nvpipe)

add_subdirectory(test)

