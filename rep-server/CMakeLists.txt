# Need to include staged files and libs
include_directories(${PROJECT_SOURCE_DIR}/rep-server/include)
#include_directories(${PROJECT_BINARY_DIR})


set(REPSRC
	../common/cpp/src/config.cpp
	src/main.cpp
)

add_executable(rep-server ${REPSRC})
add_dependencies(rep-server ftlnet)

#target_include_directories(cv-node PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(rep-server Threads::Threads ${OpenCV_LIBS} glog::glog ftlnet)


