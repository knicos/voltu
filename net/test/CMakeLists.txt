include(CTest)
enable_testing()

add_executable(rpc_test EXCLUDE_FROM_ALL
	./tests.cpp
	./rpc.cpp
	../src/dispatcher.cpp
	../src/socket.cpp
)
target_include_directories(rpc_test PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(rpc_test uriparser glog)

add_executable(protocol_unit EXCLUDE_FROM_ALL
	./tests.cpp
	./protocol_unit.cpp
)
target_include_directories(protocol_unit PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(protocol_unit glog)

add_executable(socket_unit EXCLUDE_FROM_ALL
	./tests.cpp
	./socket_unit.cpp
)
target_include_directories(socket_unit PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(socket_unit uriparser glog)

add_executable(net_integration EXCLUDE_FROM_ALL
	./tests.cpp
	./net_integration.cpp
	../src/socket.cpp
	../src/dispatcher.cpp
	../src/listener.cpp
	../src/protocol.cpp
	../src/net.cpp
)
target_include_directories(net_integration PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(net_integration uriparser glog)

add_executable(socket_test EXCLUDE_FROM_ALL
	./tests.cpp
	./net_raw.cpp
	../src/net.cpp
	../src/socket.cpp
	../src/listener.cpp
	./ice.cpp
	../src/ice.cpp
	./uri.cpp
	../src/dispatcher.cpp
)
target_include_directories(socket_test PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(socket_test uriparser glog)

add_custom_target(tests)
add_dependencies(tests rpc_test socket_test socket_unit protocol_unit net_integration)

