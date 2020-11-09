find_package(Python3 COMPONENTS Interpreter)

function(add_python_test TEST_NAME TEST_SCRIPT)
    add_test(NAME ${TEST_NAME}
        COMMAND Python3::Interpreter -B -m unittest ${TEST_SCRIPT}
        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
    # binary module directory to PYTHONPATH
    set_tests_properties(${TEST_NAME} PROPERTIES
        ENVIRONMENT PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}/..)
endfunction()

add_python_test(TestLoad test_load.py)
