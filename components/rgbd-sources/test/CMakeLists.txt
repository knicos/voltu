### Source Unit ################################################################
add_executable(source_unit
$<TARGET_OBJECTS:CatchTest>
	../src/frame.cpp
	./source_unit.cpp
)
target_include_directories(source_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(source_unit
	ftlcommon ftlcodecs ftlnet Eigen3::Eigen ftldata ${CUDA_LIBRARIES})

add_test(SourceUnitTest source_unit)

### Frame Unit #################################################################
add_executable(frame_unit
$<TARGET_OBJECTS:CatchTest>
	./frame_unit.cpp
	../src/frame.cpp
)
target_include_directories(frame_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(frame_unit
	ftlcommon ftlcodecs ftldata)

add_test(FrameUnitTest frame_unit)
