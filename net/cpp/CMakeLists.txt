# Need to include staged files and libs
include_directories(${PROJECT_SOURCE_DIR}/net/cpp/include)
#include_directories(${PROJECT_BINARY_DIR})


add_library(ftlnet
	src/listener.cpp
	src/peer.cpp
	src/dispatcher.cpp
	src/universe.cpp
	src/ws_internal.cpp
)

check_function_exists(uriParseSingleUriA HAVE_URIPARSESINGLE)

target_include_directories(ftlnet PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
	PRIVATE src)
target_link_libraries(ftlnet Threads::Threads glog::glog)

install(TARGETS ftlnet EXPORT ftlnet-config
	ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

add_executable(net-cli src/main.cpp ../../common/cpp/src/config.cpp)
target_link_libraries(net-cli ftlnet glog::glog ${URIPARSER_LIBRARIES} Threads::Threads ${READLINE_LIBRARY} ${UUID_LIBRARIES})
add_dependencies(net-cli ftlnet)

ADD_SUBDIRECTORY(test)
