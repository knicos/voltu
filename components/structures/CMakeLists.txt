
add_library(ftldata ./src/new_frame.cpp ./src/pool.cpp ./src/frameset.cpp ./src/creators.cpp)

target_include_directories(ftldata PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(ftldata ftlcommon Eigen3::Eigen ftlcodecs)

target_precompile_headers(ftldata
	PRIVATE ../common/cpp/include/ftl/utility/msgpack_optional.hpp
	PRIVATE ../common/cpp/include/ftl/cuda_common.hpp
	PRIVATE ../common/cpp/include/loguru.hpp
	PRIVATE include/ftl/data/new_frame.hpp
	PRIVATE include/ftl/data/new_frameset.hpp
)

set_property(TARGET ftldata PROPERTY CUDA_ARCHITECTURES OFF)

if (BUILD_TESTS)
add_subdirectory(test)
endif()

