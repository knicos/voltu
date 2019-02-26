add_executable(mapped_ptr_unit EXCLUDE_FROM_ALL
	./tests.cpp
	./mapped_ptr_unit.cpp
	../src/blob.cpp
)
target_link_libraries(mapped_ptr_unit gflags glog)

add_executable(cluster_unit EXCLUDE_FROM_ALL
	./tests.cpp
	../src/cluster.cpp
	./cluster_unit.cpp
)
target_link_libraries(cluster_unit uriparser ftlnet gflags glog)

add_executable(peer_test EXCLUDE_FROM_ALL
	./peer_cli.cpp
	../src/p2prm.cpp
	../src/cluster.cpp
	../src/blob.cpp
)
target_link_libraries(peer_test uriparser ftlnet gflags glog)

add_test(Mapped_ptrUnitTest mapped_ptr_unit)
add_test(ClusterUnitTest cluster_unit)

add_custom_target(tests)
add_dependencies(tests mapped_ptr_unit cluster_unit peer_test)

