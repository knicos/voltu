find_package(Python3 COMPONENTS Interpreter)

function(add_python_test TEST_NAME TEST_SCRIPT)
	add_test(NAME ${TEST_NAME}
		COMMAND Python3::Interpreter -B -m unittest ${TEST_SCRIPT}
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

	# binary module directory to PYTHONPATH, additional variables have to be
	# separated by semicolon
	set_tests_properties(${TEST_NAME} PROPERTIES ENVIRONMENT
		"PYTHONPATH=${SDK_BINARY_DIR}/python;LD_LIBRARY_PATH=${CMAKE_CURRENT_BINARY_DIR}/..")
	set_property(TEST ${TEST_NAME} APPEND PROPERTY DEPENDS voltu_sdk)
endfunction()

add_python_test(Py_TestLoad test_load.py)
