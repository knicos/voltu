### Configurable Unit ################################################################
add_executable(configurable_unit
	./tests.cpp
	../src/configurable.cpp
	../src/uri.cpp
	../src/config.cpp
	../src/configuration.cpp
	../src/loguru.cpp
	../src/ctpl_stl.cpp
	../src/cuda_common.cpp
	./configurable_unit.cpp
)
target_include_directories(configurable_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(configurable_unit
	${URIPARSER_LIBRARIES}
	Threads::Threads ${OS_LIBS} ${OpenCV_LIBS} ${CUDA_LIBRARIES})

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

### Timer Unit ################################################################
add_executable(timer_unit
	./tests.cpp
	../src/timer.cpp
	../src/config.cpp
	../src/loguru.cpp
	../src/ctpl_stl.cpp
	./timer_unit.cpp
)
target_include_directories(timer_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(timer_unit
	Threads::Threads ${OS_LIBS})


add_test(ConfigurableUnitTest configurable_unit)
add_test(URIUnitTest uri_unit)
# add_test(TimerUnitTest timer_unit) CI server can't achieve this


