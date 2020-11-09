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

    DEPENDS voltu_sdk gen.py
)

pybind11_add_module(voltu_sdk_py MODULE
    automatic_bindings.cpp
    module.cpp
)

target_include_directories(voltu_sdk_py PUBLIC include)
target_include_directories(voltu_sdk_py PRIVATE .)

target_link_libraries(voltu_sdk_py PUBLIC voltu_sdk)
set_target_properties(voltu_sdk_py PROPERTIES OUTPUT_NAME voltu)

if (WITH_TESTS)
    add_subdirectory(tests)
endif()
