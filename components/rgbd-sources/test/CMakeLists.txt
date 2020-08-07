### Source Unit ################################################################
add_executable(source_unit
$<TARGET_OBJECTS:CatchTest>
	../src/frame.cpp
	./source_unit.cpp
)
target_include_directories(source_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(source_unit
	ftlcommon ftlcalibration ftlcodecs ftlnet Eigen3::Eigen ftldata ${CUDA_LIBRARIES})

add_test(SourceUnitTest source_unit)

