add_executable(tests EXCLUDE_FROM_ALL
	./tests.cpp
	./net_raw.cpp
	../src/raw.cpp
	./ice.cpp
	../src/ice.cpp
	./uri.cpp
)
target_include_directories(tests PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(tests uriparser)

