# Need to include staged files and libs
#include_directories(${PROJECT_SOURCE_DIR}/net/cpp/include)
#include_directories(${PROJECT_BINARY_DIR})


add_library(ftlnet
	src/listener.cpp
	src/peer.cpp
	src/dispatcher.cpp
	src/universe.cpp
	src/ws_internal.cpp
	src/net_configurable.cpp
)

target_include_directories(ftlnet PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)
target_link_libraries(ftlnet ftlctrl ftlcommon Threads::Threads glog::glog ${UUID_LIBRARIES})

target_precompile_headers(ftlnet REUSE_FROM ftlcommon)

#install(TARGETS ftlnet EXPORT ftlnet-config
#	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
#	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
#install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} COMPONENT development)


if (BUILD_TESTS)
add_subdirectory(test)
endif()
