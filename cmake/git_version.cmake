find_package(Git QUIET REQUIRED)

if (Git_FOUND)
	#CHECK_REQUIRED_VARIABLE(GIT_EXECUTABLE)

	execute_process(COMMAND
        "${GIT_EXECUTABLE}" describe --tags
        WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
        RESULT_VARIABLE GIT_RESULT
        OUTPUT_VARIABLE VERSION
        ERROR_QUIET
		OUTPUT_STRIP_TRAILING_WHITESPACE)
		
	execute_process(COMMAND
        "${GIT_EXECUTABLE}" rev-parse --abbrev-ref HEAD
        WORKING_DIRECTORY "${PROJECT_SOURCE_DIR}"
        RESULT_VARIABLE GIT_RESULT
        OUTPUT_VARIABLE BRANCH
        ERROR_QUIET
        OUTPUT_STRIP_TRAILING_WHITESPACE)
        
	#string(REGEX REPLACE "^v([0-9]+)\\..*" "\\1" ftl_VERSION_MAJOR "${VERSION}")
	#string(REGEX REPLACE "^v[0-9]+\\.([0-9]+).*" "\\1" ftl_VERSION_MINOR "${VERSION}")
	#string(REGEX REPLACE "^v[0-9]+\\.[0-9]+\\.([0-9]+).*" "\\1" ftl_VERSION_PATCH "${VERSION}")
	#string(REGEX REPLACE "^v[0-9]+\\.[0-9]+\\.[0-9]+-([0-9]+).*" "\\1" ftl_VERSION_COMMITS "${VERSION}")
	#string(REGEX REPLACE "^v[0-9]+\\.[0-9]+\\.[0-9]+-[0-9]+-(.*)" "\\1" ftl_VERSION_SHA1 "${VERSION}")
	#set(FTL_VERSION "\"${ftl_VERSION_MAJOR}.${ftl_VERSION_MINOR}.${ftl_VERSION_PATCH}\"")

else()
	set(VERSION "unknown")
	#set(ftl_VERSION_MAJOR "0")
	#set(ftl_VERSION_MINOR "0")
	#set(ftl_VERSION_PATCH "0")
	#set(FTL_VERSION "\"${ftl_VERSION_MAJOR}.${ftl_VERSION_MINOR}.${ftl_VERSION_PATCH}\"")
	
	message(WARNING "Version could not be obtained from git")

endif()


