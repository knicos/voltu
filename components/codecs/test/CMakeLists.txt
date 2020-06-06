### OpenCV Codec Unit ################################################################
add_executable(opencv_codec_unit
$<TARGET_OBJECTS:CatchTest>
	../src/bitrates.cpp
	../src/encoder.cpp
	../src/opencv_encoder.cpp
	../src/opencv_decoder.cpp
	./opencv_codec_unit.cpp
)
target_include_directories(opencv_codec_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(opencv_codec_unit
	Threads::Threads ${OS_LIBS} ${OpenCV_LIBS} ftlcommon)


add_test(OpenCVCodecUnitTest opencv_codec_unit)


### Nvidia Codec Unit ################################################################
add_executable(nvidia_codec_unit
$<TARGET_OBJECTS:CatchTest>
	../src/bitrates.cpp
	../src/encoder.cpp
	#../src/nvpipe_encoder.cpp
	#../src/nvpipe_decoder.cpp
	$<TARGET_OBJECTS:NvidiaCodec>
	../src/depth_convert.cu
	./nvidia_codec_unit.cpp
)
if (WIN32)
	if (CMAKE_SIZEOF_VOID_P EQUAL 8)
		target_link_directories(nvidia_codec_unit PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../src/Video_Codec_SDK_9.1.23/Lib/x64)
	elseif (CMAKE_SIZEOF_VOID_P EQUAL 4)
		target_link_directories(nvidia_codec_unit PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../src/Video_Codec_SDK_9.1.23/Lib/Win32)
	endif()
endif()
target_include_directories(nvidia_codec_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include"
${CMAKE_CURRENT_SOURCE_DIR}/../src/Video_Codec_SDK_9.1.23/include
${CMAKE_CURRENT_SOURCE_DIR}/../src/Video_Codec_SDK_9.1.23/Samples/NvCodec)
target_link_libraries(nvidia_codec_unit
	Threads::Threads ${OS_LIBS} ${OpenCV_LIBS} ${CUDA_LIBRARIES} ftlcommon nvcuvid cuda)


add_test(NvidiaCodecUnitTest nvidia_codec_unit)

### Reader Writer Unit ################################################################
#add_executable(rw_unit
#	./tests.cpp
#	../src/writer.cpp
#	../src/reader.cpp
#	./readwrite_test.cpp
#)
#target_include_directories(rw_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
#target_link_libraries(rw_unit
#	Threads::Threads ${OS_LIBS} ${OpenCV_LIBS} ${CUDA_LIBRARIES} ftlcommon Eigen3::Eigen)


#add_test(RWUnitTest rw_unit)

### Channel Unit ###############################################################
add_executable(channel_unit
$<TARGET_OBJECTS:CatchTest>
	./channel_unit.cpp
)
target_include_directories(channel_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(channel_unit
	ftlcommon)

add_test(ChannelUnitTest channel_unit)
