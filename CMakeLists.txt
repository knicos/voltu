cmake_minimum_required (VERSION 3.1.0)
include (CheckIncludeFile)
include (CheckIncludeFileCXX)
include (CheckFunctionExists)
include(CheckLanguage)

project (ftl.utu.fi)

include(CTest)
enable_testing()

set(THREADS_PREFER_PTHREAD_FLAG ON)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

find_package( glog REQUIRED )
find_package( OpenCV REQUIRED )
find_package( Threads REQUIRED )
find_package( URIParser REQUIRED )
find_package( MsgPack REQUIRED )
find_package( LibSGM )

if (NOT WIN32)
find_library( READLINE_LIBRARY NAMES readline libreadline )
if (NOT READLINE_LIBRARY)
	message(FATAL "Readline library missing")
else()
	message(STATUS "Found Readline: ${READLINE_LIBRARY}")
endif()
endif()

find_program( NODE_NPM NAMES npm )
if (NODE_NPM)
	message(STATUS "Found NPM: ${NODE_NPM}")
endif()

include_directories(${GLOG_INCLUDE_DIRS})

# Why is this problematic on some machines?
check_language(CUDA)
if (CUDA_TOOLKIT_ROOT_DIR)
enable_language(CUDA)
set(CMAKE_CUDA_FLAGS "")
set(CMAKE_CUDA_FLAGS_DEBUG "-g -DDEBUG -D_DEBUG -Wall")
set(CMAKE_CUDA_FLAGS_RELEASE "")
set(HAVE_CUDA TRUE)
include_directories(${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})

set_property(TARGET Threads::Threads
                 PROPERTY INTERFACE_COMPILE_OPTIONS $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler -pthread>
                                                    "$<$<NOT:$<COMPILE_LANGUAGE:CUDA>>:-pthread>")

endif ()

if (NOT MsgPack_FOUND)
	check_include_file_cxx("msgpack.hpp" MSGPACK_FOUND)
	if (NOT MSGPACK_FOUND)
		message(ERROR "Msgpack is required")
	endif()
else()
	if(WIN32)
		# Find include
		find_path(MSGPACK_INCLUDE_DIRS
		    NAMES msgpack.hpp
		    PATHS "C:/Program Files/msgpack" "C:/Program Files (x86)/msgpack"
		    PATH_SUFFIXES include
		)
	endif()
	include_directories(${MSGPACK_INCLUDE_DIRS})
endif()

if (NOT WIN32)
check_include_file("uuid/uuid.h" UUID_FOUND)
if (NOT UUID_FOUND)
	message(ERROR "UUID library is required")
endif()
find_library(UUID_LIBRARIES NAMES uuid libuuid)
else()

endif()

find_program(CPPCHECK_FOUND cppcheck)
if (CPPCHECK_FOUND)
	message(STATUS "Found cppcheck: will perform source checks")
	set(CMAKE_CXX_CPPCHECK "cppcheck" "--enable=style" "--suppress=*:*catch.hpp" "--suppress=*:*json.hpp")
endif()

include_directories(${PROJECT_SOURCE_DIR}/common/cpp/include)

include(git_version)

if (WIN32)
	file(TO_CMAKE_PATH "$ENV{USERPROFILE}" ENV_USERPROFILE)
	set(FTL_LOCAL_CONFIG_ROOT "\"${ENV_USERPROFILE}/AppData/ftl\"")
	message(STATUS "Config root is ${FTL_LOCAL_CONFIG_ROOT}")
	set(FTL_LOCAL_CACHE_ROOT "\"$ENV{USERPROFILE}/AppData/ftl\"")
	set(FTL_LOCAL_DATA_ROOT "\"$ENV{USERPROFILE}/AppData/ftl\"")
	add_definitions(-DWIN32)

	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++17")
	set(CMAKE_CXX_FLAGS_DEBUG "-D_DEBUG -Wall")
	set(CMAKE_CXX_FLAGS_RELEASE "/O2")
endif (WIN32)

if (UNIX)
	add_definitions(-DUNIX)
	
	set(FTL_LOCAL_CONFIG_ROOT "\"$ENV{HOME}/.config/ftl\"")
	message(STATUS "Config root is ${FTL_LOCAL_CONFIG_ROOT}")
	set(FTL_LOCAL_CACHE_ROOT "\"$ENV{HOME}/.cache/ftl\"")
	set(FTL_LOCAL_DATA_ROOT "\"$ENV{HOME}/.local/share/ftl\"")
	
	set(FTL_GLOBAL_CONFIG_ROOT "\"${CMAKE_INSTALL_PREFIX}/share/ftl\"")
	message(STATUS "Global config root is ${FTL_GLOBAL_CONFIG_ROOT}")
	set(FTL_GLOBAL_CACHE_ROOT "\"${CMAKE_INSTALL_PREFIX}/share/ftl\"")
	set(FTL_GLOBAL_DATA_ROOT "\"${CMAKE_INSTALL_PREFIX}/share/ftl\"")

	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -pg -Wall")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -msse3 -mfpmath=sse")
endif (UNIX)

SET(CMAKE_USE_RELATIVE_PATHS ON)

add_subdirectory(net)
add_subdirectory(cv-node)

configure_file(${CMAKE_SOURCE_DIR}/common/cpp/include/ftl/config.h.in
               ${CMAKE_SOURCE_DIR}/common/cpp/include/ftl/config.h
)

configure_file(${CMAKE_SOURCE_DIR}/common/cpp/src/config.cpp.in
               ${CMAKE_SOURCE_DIR}/common/cpp/src/config.cpp
)

