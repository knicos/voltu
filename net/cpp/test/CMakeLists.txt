add_executable(protocol_unit
	./tests.cpp
	./protocol_unit.cpp
)
target_link_libraries(protocol_unit glog::glog)

add_executable(socket_unit
	./tests.cpp
	../src/ws_internal.cpp
	./socket_unit.cpp
)
target_include_directories(socket_unit PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(socket_unit ${URIPARSER_LIBRARIES} glog::glog)

add_executable(uri_unit
	./tests.cpp
	./uri_unit.cpp
)

target_link_libraries(uri_unit ${URIPARSER_LIBRARIES})

add_executable(p2p_base_unit
	./tests.cpp
	./p2p_base_unit.cpp
	../src/p2p.cpp
	../src/socket.cpp
	../src/dispatcher.cpp
	../src/protocol.cpp
	../src/net.cpp
	../src/listener.cpp
	../src/ws_internal.cpp
)

target_link_libraries(p2p_base_unit ${URIPARSER_LIBRARIES} gflags glog::glog ${UUID_LIBRARIES})

add_executable(net_integration
	./tests.cpp
	./net_integration.cpp
	../src/socket.cpp
	../src/dispatcher.cpp
	../src/listener.cpp
	../src/protocol.cpp
	../src/net.cpp
	../src/ws_internal.cpp
)
target_link_libraries(net_integration ${URIPARSER_LIBRARIES} glog::glog)

add_test(URIUnitTest uri_unit)
add_test(ProtocolUnitTest protocol_unit)
add_test(SocketUnitTest socket_unit)
add_test(NetIntegrationTest net_integration)

add_custom_target(tests)
add_dependencies(tests socket_unit protocol_unit net_integration uri_unit)

