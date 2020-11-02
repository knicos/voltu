set(SDK_AUTO_HEADERS
    voltu/types/channel.hpp
    voltu/types/frame.hpp
    voltu/types/image.hpp
    voltu/types/intrinsics.hpp
    voltu/observer.hpp
    voltu/feed.hpp
    voltu/initialise.hpp
    voltu/room.hpp
    voltu/source.hpp
    voltu/system.hpp
)

add_custom_command(
    OUTPUT automatic_bindings.cpp
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/gen.py
            automatic_bindings.cpp
            ${CMAKE_CURRENT_SOURCE_DIR}/../include
            ${SDK_AUTO_HEADERS}

    DEPENDS voltu_sdk
)

pybind11_add_module(voltu_sdk_py MODULE
    automatic_bindings.cpp
    module.cpp
)

target_include_directories(voltu_sdk_py PUBLIC include)
target_include_directories(voltu_sdk_py PRIVATE .)

target_link_libraries(voltu_sdk_py PUBLIC voltu_sdk)
set_target_properties(voltu_sdk_py PROPERTIES OUTPUT_NAME voltu)

enable_testing()
find_package(Python3 COMPONENTS Interpreter)

function(add_python_test TEST_NAME TEST_SCRIPT)
    add_test(NAME ${TEST_NAME}
        COMMAND Python3::Interpreter -m unittest ${TEST_SCRIPT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endfunction()

add_python_test(test_load tests/test_load.py)
