add_library(BaseCodec OBJECT
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
	${CMAKE_CURRENT_SOURCE_DIR}/src/Video_Codec_SDK_9.1.23/include
	${CMAKE_CURRENT_SOURCE_DIR}/src/Video_Codec_SDK_9.1.23/Samples/NvCodec
	$<TARGET_PROPERTY:ftlcommon,INTERFACE_INCLUDE_DIRECTORIES>
)
set_property(TARGET BaseCodec PROPERTY CUDA_ARCHITECTURES OFF)

add_library(OpenCVCodec OBJECT	
	src/opencv_encoder.cpp
	src/opencv_decoder.cpp
)
target_include_directories(OpenCVCodec PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include
	$<TARGET_PROPERTY:ftlcommon,INTERFACE_INCLUDE_DIRECTORIES>
)

set_property(TARGET OpenCVCodec PROPERTY CUDA_ARCHITECTURES OFF)

set(CODECSRC
$<TARGET_OBJECTS:BaseCodec>
$<TARGET_OBJECTS:OpenCVCodec>
)

add_library(NvidiaCodec OBJECT	
	src/nvidia_encoder.cpp
	src/nvidia_decoder.cpp
	src/Video_Codec_SDK_9.1.23/Samples/NvCodec/NvDecoder/NvDecoder.cpp
	src/Video_Codec_SDK_9.1.23/Samples/NvCodec/NvEncoder/NvEncoder.cpp
	src/Video_Codec_SDK_9.1.23/Samples/NvCodec/NvEncoder/NvEncoderCuda.cpp
)
target_include_directories(NvidiaCodec PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${CMAKE_CURRENT_SOURCE_DIR}/src/Video_Codec_SDK_9.1.23/include
	${CMAKE_CURRENT_SOURCE_DIR}/src/Video_Codec_SDK_9.1.23/Samples/NvCodec
	$<TARGET_PROPERTY:ftlcommon,INTERFACE_INCLUDE_DIRECTORIES>
)
list(APPEND CODECSRC $<TARGET_OBJECTS:NvidiaCodec>)

set_property(TARGET NvidiaCodec PROPERTY CUDA_ARCHITECTURES OFF)

add_library(ftlcodecs ${CODECSRC})

if (WIN32)
	if (CMAKE_SIZEOF_VOID_P EQUAL 8)
		target_link_directories(ftlcodecs PUBLIC src/Video_Codec_SDK_9.1.23/Lib/x64)
	elseif (CMAKE_SIZEOF_VOID_P EQUAL 4)
		target_link_directories(ftlcodecs PUBLIC src/Video_Codec_SDK_9.1.23/Lib/Win32)
	endif()
endif()

target_include_directories(ftlcodecs PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)

target_link_libraries(ftlcodecs ftlcommon ${OpenCV_LIBS} ${CUDA_LIBRARIES} Eigen3::Eigen nvcuvid cuda)

target_precompile_headers(ftlcodecs REUSE_FROM ftlcommon)

set_property(TARGET ftlcodecs PROPERTY CUDA_ARCHITECTURES OFF)

if (BUILD_TESTS)
add_subdirectory(test)
endif()

