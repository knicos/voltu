### Stream Unit ################################################################
add_executable(stream_unit
	./tests.cpp
	./stream_unit.cpp
	../src/stream.cpp
)
target_include_directories(stream_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(stream_unit
	ftlcommon ftlcodecs ftlrgbd)

add_test(StreamUnitTest stream_unit)

### File Stream Unit ###########################################################
add_executable(filestream_unit
	./tests.cpp
	./filestream_unit.cpp
	../src/filestream.cpp
	../src/stream.cpp
)
target_include_directories(filestream_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(filestream_unit
	ftlcommon ftlcodecs ftlrgbd)

add_test(FileStreamUnitTest filestream_unit)

### Net Stream Unit ###########################################################
#add_executable(netstream_unit
#	./tests.cpp
#	./netstream_unit.cpp
#	../src/stream.cpp
#)
#target_include_directories(netstream_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
#target_link_libraries(netstream_unit
#	ftlcommon ftlcodecs ftlrgbd ftlnet)

#add_test(NetStreamUnitTest netstream_unit)
