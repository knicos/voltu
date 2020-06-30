### DBSCAN Unit ################################################################
add_executable(dbscan_unit
	$<TARGET_OBJECTS:CatchTest>
	./dbscan_unit.cpp
)
target_include_directories(dbscan_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(dbscan_unit ftlcommon)

add_test(DBSCANUnitTest dbscan_unit)
