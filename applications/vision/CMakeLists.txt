set(CVNODESRC
	src/main.cpp
)

add_executable(voltu-vision ${CVNODESRC})
install(TARGETS voltu-vision RUNTIME COMPONENT vision)

if (CUDA_FOUND)
set_property(TARGET voltu-vision PROPERTY CUDA_SEPARABLE_COMPILATION OFF)
endif()

target_link_libraries(voltu-vision ftlrgbd ftlcommon ftlstreams ftlctrl ${OpenCV_LIBS} ${CUDA_LIBRARIES} ftlnet ftlaudio)

target_precompile_headers(voltu-vision REUSE_FROM ftldata)
