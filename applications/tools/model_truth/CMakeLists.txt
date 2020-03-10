set(GTSRC
	src/main.cpp
)

add_executable(model-truth ${GTSRC})

target_include_directories(model-truth PUBLIC
	#$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	#$<INSTALL_INTERFACE:include>
	PRIVATE src)

if (CUDA_FOUND)
set_property(TARGET model-truth PROPERTY CUDA_SEPARABLE_COMPILATION ON)
endif()

#target_include_directories(cv-node PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(model-truth ftlcommon ftlrgbd Threads::Threads ${OpenCV_LIBS} ftlctrl ftlnet ftlrender ftloperators ftlstreams assimp)
