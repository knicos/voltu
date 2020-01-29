add_library(BaseCodec OBJECT
	src/bitrates.cpp
	src/encoder.cpp
	src/decoder.cpp
	src/generate.cpp
	src/writer.cpp
	src/reader.cpp
	src/channels.cpp
	src/depth_convert.cu
)
target_include_directories(BaseCodec PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include
	$<TARGET_PROPERTY:ftlcommon,INTERFACE_INCLUDE_DIRECTORIES>
	$<TARGET_PROPERTY:nvpipe,INTERFACE_INCLUDE_DIRECTORIES>
)

add_library(OpenCVCodec OBJECT	
	src/opencv_encoder.cpp
	src/opencv_decoder.cpp
)
target_include_directories(OpenCVCodec PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include
	$<TARGET_PROPERTY:ftlcommon,INTERFACE_INCLUDE_DIRECTORIES>
)

set(CODECSRC
$<TARGET_OBJECTS:BaseCodec>
$<TARGET_OBJECTS:OpenCVCodec>
)

if (HAVE_NVPIPE)
	add_library(NvPipeCodec OBJECT	
		src/nvpipe_encoder.cpp
		src/nvpipe_decoder.cpp
	)
	target_include_directories(NvPipeCodec PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/include
		$<TARGET_PROPERTY:ftlcommon,INTERFACE_INCLUDE_DIRECTORIES>
		$<TARGET_PROPERTY:nvpipe,INTERFACE_INCLUDE_DIRECTORIES>
	)
	list(APPEND CODECSRC $<TARGET_OBJECTS:NvPipeCodec>)
endif()

add_library(ftlcodecs ${CODECSRC})

target_include_directories(ftlcodecs PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)

#target_include_directories(cv-node PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(ftlcodecs ftlcommon ${OpenCV_LIBS} ${CUDA_LIBRARIES} Eigen3::Eigen nvpipe)

if (BUILD_TESTS)
add_subdirectory(test)
endif()

