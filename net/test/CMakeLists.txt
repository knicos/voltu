add_executable(tests EXCLUDE_FROM_ALL
	./tests.cpp
	./net_raw.cpp
	../src/net.cpp
	../src/socket.cpp
	../src/listener.cpp
	./ice.cpp
	../src/ice.cpp
	./uri.cpp
	./rpc.cpp
	../src/dispatcher.cpp
)
target_include_directories(tests PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(tests uriparser)

