### Configurable Unit ################################################################
add_executable(configurable_unit
	./tests.cpp
	../src/configurable.cpp
	./configurable_unit.cpp
)
target_include_directories(configurable_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(configurable_unit
	${URIPARSER_LIBRARIES}
	glog::glog)



add_test(ConfigurableUnitTest configurable_unit)


