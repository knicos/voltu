set(MIDSRC
	src/main.cpp
)

add_executable(middlebury-gen ${MIDSRC})

target_include_directories(middlebury-gen PUBLIC
	#$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	#$<INSTALL_INTERFACE:include>
	PRIVATE src)

if (CUDA_FOUND)
set_property(TARGET middlebury-gen PROPERTY CUDA_SEPARABLE_COMPILATION ON)
endif()

#target_include_directories(cv-node PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(middlebury-gen ftlcommon ftlrgbd Threads::Threads ${OpenCV_LIBS} ftlrender ftloperators ftlstreams)
