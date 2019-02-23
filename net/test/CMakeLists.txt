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
add_dependencies(tests rpc_test socket_test)

