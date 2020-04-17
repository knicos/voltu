#add_library(CatchTest OBJECT ./tests.cpp)

add_executable(dsi_cpu_unit
$<TARGET_OBJECTS:CatchTest>
	./dsi_unit.cpp
)
target_include_directories(dsi_cpu_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_include_directories(dsi_cpu_unit PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(dsi_cpu_unit Threads::Threads ${OpenCV_LIBS})

add_test(DSICPUUnitTest dsi_cpu_unit)

add_executable(dsi_gpu_unit
$<TARGET_OBJECTS:CatchTest>
	./dsi_gpu_unit.cu
)
target_include_directories(dsi_gpu_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_include_directories(dsi_gpu_unit PUBLIC ${OpenCV_INCLUDE_DIRS})
target_compile_definitions(dsi_gpu_unit PUBLIC USE_GPU)
target_link_libraries(dsi_gpu_unit Threads::Threads ${OpenCV_LIBS})

add_test(DSIGPUUnitTest dsi_gpu_unit)

add_executable(array2d_unit
$<TARGET_OBJECTS:CatchTest>
	./array2d_unit.cpp
)
target_include_directories(array2d_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_include_directories(array2d_unit PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(array2d_unit Threads::Threads ${OpenCV_LIBS})

add_test(Array2DUnitTest array2d_unit)

add_executable(matching_cost_unit
$<TARGET_OBJECTS:CatchTest>
	../src/costs/census.cu
	../src/costs/gradient.cu
	../src/costs/mutual_information.cu
	./matching_cost_unit.cpp
)
target_include_directories(matching_cost_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include" "${CMAKE_CURRENT_SOURCE_DIR}/../src")
target_include_directories(matching_cost_unit PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(matching_cost_unit Threads::Threads ${OpenCV_LIBS})

add_test(MatchingCostUnitTest matching_cost_unit)

add_executable(aggregation_unit
$<TARGET_OBJECTS:CatchTest>
	./aggregation_unit.cpp
)
target_include_directories(aggregation_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_include_directories(aggregation_unit PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(aggregation_unit Threads::Threads ${OpenCV_LIBS})

add_test(AggregationUnitTest aggregation_unit)

add_executable(wta_unit
$<TARGET_OBJECTS:CatchTest>
	./wta_unit.cpp
)
target_include_directories(wta_unit PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../include")
target_include_directories(wta_unit PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(wta_unit Threads::Threads ${OpenCV_LIBS})

add_test(WTAUnitTest wta_unit)
