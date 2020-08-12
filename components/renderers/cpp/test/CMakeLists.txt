### Renderer Unit ##############################################################
add_executable(render_unit
	$<TARGET_OBJECTS:CatchTest>
	./render_unit.cpp
)
target_include_directories(render_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(render_unit
	ftlcommon ftlcodecs ftldata ftlrgbd)

	target_precompile_headers(render_unit REUSE_FROM ftlcommon)

add_test(RenderUnitTest render_unit)