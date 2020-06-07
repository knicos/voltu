### Frame Unit #################################################################
add_executable(nframe_unit
	$<TARGET_OBJECTS:CatchTest>
	./frame_unit.cpp
	../src/new_frame.cpp
)
target_include_directories(nframe_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(nframe_unit
	ftlcommon ftlcodecs)