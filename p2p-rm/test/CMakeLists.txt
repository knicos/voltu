include(CTest)
enable_testing()

add_executable(remote_ptr EXCLUDE_FROM_ALL
	./tests.cpp
	./remote_ptr.cpp
)

add_executable(p2p_rm EXCLUDE_FROM_ALL
	./tests.cpp
	../src/p2prm.cpp
	../src/blob.cpp
	./p2p-rm.cpp
)
target_link_libraries(p2p_rm uriparser)

add_test(Remote_ptr remote_ptr)
add_test(RM_API p2p_rm)

add_custom_target(tests)
add_dependencies(tests remote_ptr p2p_rm)

