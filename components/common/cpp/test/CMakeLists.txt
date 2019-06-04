### Configurable Unit ################################################################
add_executable(configurable_unit
	./tests.cpp
	../src/configurable.cpp
	../src/uri.cpp
	../src/config.cpp
	../src/configuration.cpp
	../src/loguru.cpp
	./configurable_unit.cpp
)
target_include_directories(configurable_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(configurable_unit
	${URIPARSER_LIBRARIES}
	Threads::Threads ${OS_LIBS})

### URI ########################################################################
add_executable(uri_unit
	./tests.cpp
	../src/uri.cpp
	../src/loguru.cpp
	./uri_unit.cpp)
target_include_directories(uri_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(uri_unit
	Threads::Threads ${OS_LIBS}
	${URIPARSER_LIBRARIES})



add_test(ConfigurableUnitTest configurable_unit)
add_test(URIUnitTest uri_unit)


