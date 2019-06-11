add_subdirectory(cpp)

# Perform Javascript module tests
if (NODE_NPM)
#	add_test(
#		NAME NetJSTests
#		COMMAND npm test
#		WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/net/js
#	)
endif()

