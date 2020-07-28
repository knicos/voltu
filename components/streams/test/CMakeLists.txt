### Stream Unit ################################################################
add_executable(stream_unit
	$<TARGET_OBJECTS:CatchTest>
	./stream_unit.cpp
	../src/stream.cpp
)
target_include_directories(stream_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(stream_unit
	ftlcommon ftlcodecs ftlrgbd)

target_precompile_headers(stream_unit REUSE_FROM ftldata)

add_test(StreamUnitTest stream_unit)

### File Stream Unit ###########################################################
add_executable(filestream_unit
$<TARGET_OBJECTS:CatchTest>
	./filestream_unit.cpp
	../src/filestream.cpp
	../src/stream.cpp
)
target_include_directories(filestream_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(filestream_unit
	ftlcommon ftlcodecs ftlrgbd)

target_precompile_headers(filestream_unit REUSE_FROM ftldata)

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

### Sender Unit ################################################################
add_executable(sender_unit
$<TARGET_OBJECTS:CatchTest>
	./sender_unit.cpp
	../src/sender.cpp
	../src/stream.cpp
	../src/injectors.cpp
	../src/parsers.cpp
)
target_include_directories(sender_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(sender_unit
	ftlcommon ftlcodecs ftlrgbd ftlaudio)

target_precompile_headers(sender_unit REUSE_FROM ftldata)

add_test(SenderUnitTest sender_unit)

### Receiver Unit ##############################################################
add_executable(receiver_unit
$<TARGET_OBJECTS:CatchTest>
	./receiver_unit.cpp
	../src/receiver.cpp
	../src/stream.cpp
	../src/injectors.cpp
	../src/parsers.cpp
	../src/builder.cpp
)
target_include_directories(receiver_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(receiver_unit
	ftlcommon ftlcodecs ftlrgbd ftlaudio)

target_precompile_headers(receiver_unit REUSE_FROM ftldata)

add_test(ReceiverUnitTest receiver_unit)

### Receiver Sender Unit #######################################################
add_executable(recsend_unit
$<TARGET_OBJECTS:CatchTest>
	./recsend_unit.cpp
	../src/receiver.cpp
	../src/stream.cpp
	../src/sender.cpp
	../src/builder.cpp
)
target_include_directories(recsend_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(recsend_unit
	ftlcommon ftlcodecs ftlrgbd ftlaudio)

target_precompile_headers(recsend_unit REUSE_FROM ftldata)

add_test(RecSendUnitTest recsend_unit)

### Builder Unit ###############################################################
add_executable(builder_unit
$<TARGET_OBJECTS:CatchTest>
	./builder_unit.cpp
	../src/builder.cpp
)
target_include_directories(builder_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(builder_unit
	ftlcommon ftldata)

target_precompile_headers(builder_unit REUSE_FROM ftldata)

add_test(BuilderUnitTest builder_unit)


### Feed Unit ##################################################################
add_executable(feed_unit
	$<TARGET_OBJECTS:CatchTest>
	./feed_unit.cpp
	../src/feed.cpp
)
target_include_directories(feed_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_link_libraries(feed_unit
	ftlrgbd ftlstreams ftloperators ftlcommon ftldata)

target_precompile_headers(feed_unit REUSE_FROM ftldata)

add_test(FeedUnitTest feed_unit)
