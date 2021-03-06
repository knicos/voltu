### Socket Unit ################################################################
add_executable(peer_unit
$<TARGET_OBJECTS:CatchTest>
	../src/ws_internal.cpp
	../src/dispatcher.cpp
	./peer_unit.cpp
	../../../common/cpp/src/config.cpp
)
target_include_directories(peer_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include" "${CMAKE_CURRENT_SOURCE_DIR}/../../../common/cpp/include")
target_link_libraries(peer_unit
	ftlcommon
	glog::glog
	Threads::Threads
	${UUID_LIBRARIES})

### Net Integration ############################################################
add_executable(net_integration
$<TARGET_OBJECTS:CatchTest>
	../../../common/cpp/src/config.cpp
	./net_integration.cpp)
add_dependencies(net_integration ftlnet)
target_include_directories(net_integration PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}../include")
target_link_libraries(net_integration
	ftlnet
	ftlcommon
	glog::glog
	Threads::Threads
	${UUID_LIBRARIES})

### NetConfigurable Unit #######################################################
add_executable(net_configurable_unit
$<TARGET_OBJECTS:CatchTest>
	./net_configurable_unit.cpp)
target_link_libraries(net_configurable_unit
	ftlnet)



#add_test(ProtocolUnitTest protocol_unit)
add_test(PeerUnitTest peer_unit)
add_test(NetIntegrationTest net_integration)
# Testing of NetConfigurable is disabled.
#add_test(NetConfigurableUnitTest net_configurable_unit)

add_custom_target(tests)
add_dependencies(tests peer_unit uri_unit)

