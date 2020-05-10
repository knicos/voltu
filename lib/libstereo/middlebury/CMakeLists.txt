add_executable (middlebury
                main.cpp
                middlebury.cpp
)

target_include_directories(middlebury PRIVATE ../include/)
target_include_directories(middlebury PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(middlebury libstereo pthread stdc++fs dl ${OpenCV_LIBS})
