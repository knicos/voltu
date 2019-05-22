### Socket Unit ################################################################
add_executable(peer_unit
	./tests.cpp
	../src/ws_internal.cpp
	../src/dispatcher.cpp
	../src/uri.cpp
	./peer_unit.cpp
	../../../common/cpp/src/config.cpp
)
target_include_directories(peer_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include" "${CMAKE_CURRENT_SOURCE_DIR}/../../../common/cpp/include")
target_link_libraries(peer_unit
	${URIPARSER_LIBRARIES}
	glog::glog
	Threads::Threads
	${UUID_LIBRARIES})

### URI ########################################################################
add_executable(uri_unit
	./tests.cpp
	../src/uri.cpp
	./uri_unit.cpp)
target_include_directories(uri_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(uri_unit
	${URIPARSER_LIBRARIES})

### P2P Base Unit ##############################################################
# TODO(nick) Actually make this a unit test

#add_executable(p2p_base_unit
#	./tests.cpp
#	./p2p_base_unit.cpp)
#add_dependencies(p2p_base_unit ftlnet)
#target_link_libraries(p2p_base_unit
#	ftlnet
#	${URIPARSER_LIBRARIES}
#	glog::glog
#	${UUID_LIBRARIES})

### Net Integration ############################################################
add_executable(net_integration
	./tests.cpp
	../../../common/cpp/src/config.cpp
	./net_integration.cpp)
add_dependencies(net_integration ftlnet)
target_include_directories(net_integration PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}../include")
target_link_libraries(net_integration
	ftlnet
	${URIPARSER_LIBRARIES}
	glog::glog
	Threads::Threads
	${UUID_LIBRARIES})





add_test(URIUnitTest uri_unit)
#add_test(ProtocolUnitTest protocol_unit)
add_test(PeerUnitTest peer_unit)
add_test(NetIntegrationTest net_integration)

add_custom_target(tests)
add_dependencies(tests peer_unit uri_unit)

