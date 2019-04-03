# Need to include staged files and libs
include_directories(${PROJECT_SOURCE_DIR}/net/cpp/include)
#include_directories(${PROJECT_BINARY_DIR})


set(NETSOURCE
	src/net.cpp
	src/listener.cpp
	src/socket.cpp
	src/dispatcher.cpp
	src/protocol.cpp
	src/ws_internal.cpp
	src/p2p.cpp
)

#check_include_file("uriparser/Uri.h" HAVE_URI_H)
#if(NOT HAVE_URI_H)
#	message(FATAL_ERROR "Uriparser not found")
#endif()
check_function_exists(uriParseSingleUriA HAVE_URIPARSESINGLE)

add_library(net ${NETSOURCE})
target_link_libraries(net pthread)

add_executable(net-cli src/main.cpp)
target_link_libraries(net-cli "${PROJECT_BINARY_DIR}/net/cpp/libnet.a" ${GLOG_LIBRARIES} ${URIPARSER_LIBRARIES} pthread readline uuid)
add_dependencies(net-cli net)

ADD_SUBDIRECTORY(test)
