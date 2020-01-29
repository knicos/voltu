add_library(CatchTest OBJECT ./tests.cpp)

### Configurable Unit ################################################################
add_executable(configurable_unit
	$<TARGET_OBJECTS:CatchTest>
	./configurable_unit.cpp
)
target_include_directories(configurable_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(configurable_unit ftlcommon
	${URIPARSER_LIBRARIES}
	Threads::Threads ${OS_LIBS} ${OpenCV_LIBS} ${CUDA_LIBRARIES})

### URI ########################################################################
add_executable(uri_unit
	$<TARGET_OBJECTS:CatchTest>
	./uri_unit.cpp)
target_include_directories(uri_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(uri_unit ftlcommon
	Threads::Threads ${OS_LIBS}
	${URIPARSER_LIBRARIES})

### Timer Unit ################################################################
add_executable(timer_unit
	$<TARGET_OBJECTS:CatchTest>
	./timer_unit.cpp
)
target_include_directories(timer_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(timer_unit ftlcommon
	Threads::Threads ${OS_LIBS})

### URI ########################################################################
add_executable(msgpack_unit
	$<TARGET_OBJECTS:CatchTest>
	$<TARGET_OBJECTS:Loguru>
	./msgpack_unit.cpp)
target_include_directories(msgpack_unit PUBLIC ${OpenCV_INCLUDE_DIRS} "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(msgpack_unit Threads::Threads ${OS_LIBS} ${OpenCV_LIBS})

add_test(ConfigurableUnitTest configurable_unit)
add_test(URIUnitTest uri_unit)
add_test(MsgpackUnitTest msgpack_unit)
# add_test(TimerUnitTest timer_unit) CI server can't achieve this
