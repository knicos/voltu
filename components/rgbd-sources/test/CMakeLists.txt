### Source Unit ################################################################
add_executable(source_unit
	./tests.cpp
	./source_unit.cpp
)
target_include_directories(source_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(source_unit
	ftlcommon Eigen3::Eigen ${CUDA_LIBRARIES})

add_test(SourceUnitTest source_unit)
