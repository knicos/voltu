### OpenCV Codec Unit ################################################################
add_executable(opencv_codec_unit
	./tests.cpp
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


### NvPipe Codec Unit ################################################################
add_executable(nvpipe_codec_unit
	./tests.cpp
	../src/bitrates.cpp
	../src/encoder.cpp
	../src/nvpipe_encoder.cpp
	../src/nvpipe_decoder.cpp
	../src/depth_convert.cu
	./nvpipe_codec_unit.cpp
)
target_include_directories(nvpipe_codec_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(nvpipe_codec_unit
	Threads::Threads ${OS_LIBS} ${OpenCV_LIBS} ${CUDA_LIBRARIES} ftlcommon nvpipe)


add_test(NvPipeCodecUnitTest nvpipe_codec_unit)

### Reader Writer Unit ################################################################
add_executable(rw_unit
	./tests.cpp
	../src/writer.cpp
	../src/reader.cpp
	./readwrite_test.cpp
)
target_include_directories(rw_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(rw_unit
	Threads::Threads ${OS_LIBS} ${OpenCV_LIBS} ${CUDA_LIBRARIES} ftlcommon Eigen3::Eigen)


add_test(RWUnitTest rw_unit)

### Channel Unit ###############################################################
add_executable(channel_unit
	./tests.cpp
	./channel_unit.cpp
)
target_include_directories(channel_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(channel_unit
	ftlcommon)

add_test(ChannelUnitTest channel_unit)
