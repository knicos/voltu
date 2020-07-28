### Frame Unit #################################################################
add_executable(nframe_unit
	$<TARGET_OBJECTS:CatchTest>
	./frame_unit.cpp
)
target_include_directories(nframe_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(nframe_unit
	ftlcommon ftlcodecs)

	target_precompile_headers(nframe_unit REUSE_FROM ftlcommon)

add_test(NFrameUnitTest nframe_unit)

### Frame Example 1 ############################################################
add_executable(frame_example_1
	$<TARGET_OBJECTS:CatchTest>
	../src/pool.cpp
	../src/new_frame.cpp
	./frame_example_1.cpp
)
target_include_directories(frame_example_1 PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(frame_example_1
	ftlcommon ftlcodecs)

add_test(FrameEg1Test frame_example_1)

### Pool Unit ##################################################################
add_executable(pool_unit
	$<TARGET_OBJECTS:CatchTest>
	./pool_unit.cpp
	../src/new_frame.cpp
	../src/pool.cpp
)
target_include_directories(pool_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(pool_unit
	ftlcommon ftlcodecs)

add_test(MemPoolUnitTest pool_unit)