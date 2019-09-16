### Source Unit ################################################################
add_executable(source_unit
	./tests.cpp
	./source_unit.cpp
)
target_include_directories(source_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(source_unit
	ftlcommon ftlcodecs ftlnet Eigen3::Eigen ${CUDA_LIBRARIES})

add_test(SourceUnitTest source_unit)

### Channel Unit ###############################################################
add_executable(channel_unit
	./tests.cpp
	./channel_unit.cpp
)
target_include_directories(channel_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(channel_unit
	ftlcommon)

add_test(ChannelUnitTest channel_unit)

### Frame Unit #################################################################
add_executable(frame_unit
	./tests.cpp
	./frame_unit.cpp
	../src/frame.cpp
)
target_include_directories(frame_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(frame_unit
	ftlcommon ftlcodecs)

add_test(FrameUnitTest frame_unit)
