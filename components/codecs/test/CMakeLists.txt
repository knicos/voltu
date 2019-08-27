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
	./nvpipe_codec_unit.cpp
)
target_include_directories(nvpipe_codec_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(nvpipe_codec_unit
	Threads::Threads ${OS_LIBS} ${OpenCV_LIBS} ${CUDA_LIBRARIES} ftlcommon nvpipe)


add_test(NvPipeCodecUnitTest nvpipe_codec_unit)
