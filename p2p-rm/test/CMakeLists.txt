include(CTest)
enable_testing()

add_executable(mapped_ptr EXCLUDE_FROM_ALL
	./tests.cpp
	./mapped_ptr.cpp
	../src/blob.cpp
)

add_executable(p2p_rm EXCLUDE_FROM_ALL
	./tests.cpp
	../src/p2prm.cpp
	../src/cluster.cpp
	../src/blob.cpp
	
	./p2p-rm.cpp
)
target_link_libraries(p2p_rm uriparser)

add_executable(peer_test EXCLUDE_FROM_ALL
	./peer.cpp
	../src/p2prm.cpp
	../src/cluster.cpp
	../src/blob.cpp
)
target_link_libraries(peer_test uriparser ftlnet gflags glog)

add_test(Mapped_ptr mapped_ptr)
add_test(RM_API p2p_rm)

add_custom_target(tests)
add_dependencies(tests mapped_ptr p2p_rm peer_test)

