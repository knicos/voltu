add_executable(mapped_ptr_unit
	./tests.cpp
	./mapped_ptr_unit.cpp
	../src/blob.cpp
)
target_link_libraries(mapped_ptr_unit gflags glog uuid)

add_executable(cluster_unit
	./tests.cpp
	../src/cluster.cpp
	./cluster_unit.cpp
)
target_link_libraries(cluster_unit uriparser ftlnet gflags glog uuid)

add_executable(p2p_unit
	./tests.cpp
	./p2p_unit.cpp
)
target_link_libraries(p2p_unit ftlnet uriparser gflags glog uuid)

add_executable(p2p_integration
	./tests.cpp
	../src/cluster.cpp
	../src/p2prm.cpp
	../src/blob.cpp
	./p2p_integration.cpp
)
target_link_libraries(p2p_integration uriparser ftlnet gflags glog uuid)

add_executable(peer_test
	./peer_cli.cpp
	../src/p2prm.cpp
	../src/cluster.cpp
	../src/blob.cpp
)
target_link_libraries(peer_test uriparser ftlnet gflags glog uuid)

add_test(Mapped_ptrUnitTest mapped_ptr_unit)
add_test(ClusterUnitTest cluster_unit)
add_test(P2PUnitTest p2p_unit)

add_custom_target(tests)
add_dependencies(tests mapped_ptr_unit cluster_unit peer_test p2p_integration)

