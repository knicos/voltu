# Need to include staged files and libs
include_directories(${PROJECT_SOURCE_DIR}/reconstruct/include)
#include_directories(${PROJECT_BINARY_DIR})


set(REPSRC
	../common/cpp/src/config.cpp
	src/main.cpp
)

add_executable(ftl-reconstruct ${REPSRC})
add_dependencies(ftl-reconstruct ftlnet)

#target_include_directories(cv-node PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(ftl-reconstruct Threads::Threads ZLIB::ZLIB ${OpenCV_LIBS} glog::glog ftlnet ftlrender)


