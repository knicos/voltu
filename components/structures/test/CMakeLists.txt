### Frame Unit #################################################################
add_executable(nframe_unit
	$<TARGET_OBJECTS:CatchTest>
	./frame_unit.cpp
	../src/new_frame.cpp
	./frame_example_1.cpp
)
target_include_directories(nframe_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(nframe_unit
	ftlcommon ftlcodecs)

add_test(NFrameUnitTest nframe_unit)

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