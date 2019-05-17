cmake_minimum_required (VERSION 3.1.0)
include (CheckIncludeFile)
include (CheckIncludeFileCXX)
include (CheckFunctionExists)
include(CheckLanguage)

project (ftl.utu.fi)

set(CMAKE_CODELITE_USE_TARGETS ON)
include(GNUInstallDirs)
include(CTest)
enable_testing()

option(WITH_PCL "Use PCL if available" ON)
option(WITH_FIXSTARS "Use Fixstars libSGM if available" ON)
option(BUILD_VISION "Enable the vision component" ON)
option(BUILD_RECONSTRUCT "Enable the reconstruction component" ON)
option(BUILD_RENDERER "Enable the renderer component" ON)

set(THREADS_PREFER_PTHREAD_FLAG ON)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

include(Findglog)

find_package( OpenCV REQUIRED )
find_package( Threads REQUIRED )
find_package( URIParser REQUIRED )
find_package( MsgPack REQUIRED )

if (WITH_FIXSTARS)
	find_package( LibSGM )
endif()

if(${CMAKE_VERSION} VERSION_GREATER "3.12.0") 
	cmake_policy(SET CMP0074 NEW)
endif()

if (WITH_PCL)
	find_package( PCL QUIET COMPONENTS io common visualization registration )
endif()

set(CMAKE_CXX_STANDARD 17) # For PCL/VTK https://github.com/PointCloudLibrary/pcl/issues/2686
set(HAVE_OPENCV TRUE)

if (PCL_FOUND)
	set(HAVE_PCL TRUE)
endif()

# Readline library is not required on Windows
# May also entirely remove dependence on this... it should be optional at least.
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

check_language(CUDA)
if (CUDA_TOOLKIT_ROOT_DIR)
enable_language(CUDA)
set(CMAKE_CUDA_FLAGS "")
set(CMAKE_CUDA_FLAGS_DEBUG "-g -DDEBUG -D_DEBUG")
set(CMAKE_CUDA_FLAGS_RELEASE "")
set(HAVE_CUDA TRUE)
include_directories(${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})

# Some kind of fix for nvcc and -pthread problem on Linux
set_property(TARGET Threads::Threads
                 PROPERTY INTERFACE_COMPILE_OPTIONS $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler -pthread>
                                                    "$<$<NOT:$<COMPILE_LANGUAGE:CUDA>>:-pthread>")

endif ()

# Incase find_package failed, manually look for msgpack
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
	# TODO(nick) Create imported target.
	include_directories(${MSGPACK_INCLUDE_DIRS})
endif()

# Only need libuuid on Linux, Windows has its own
if (NOT WIN32)
check_include_file("uuid/uuid.h" UUID_FOUND)
if (NOT UUID_FOUND)
	message(ERROR "UUID library is required")
endif()
find_library(UUID_LIBRARIES NAMES uuid libuuid)
else()
endif()

# Check for optional opencv components
set(CMAKE_REQUIRED_INCLUDES ${OpenCV_INCLUDE_DIRS})
check_include_file_cxx("opencv2/viz.hpp" HAVE_VIZ)
check_include_file_cxx("opencv2/cudastereo.hpp" HAVE_OPENCVCUDA)

# Optional source problem check
find_program(CPPCHECK_FOUND cppcheck)
if (CPPCHECK_FOUND)
	message(STATUS "Found cppcheck: will perform source checks")
	set(CMAKE_CXX_CPPCHECK "cppcheck" "--enable=style" "--suppress=*:*catch.hpp" "--suppress=*:*elas*" "--suppress=*:*json.hpp")
endif()

include_directories(${PROJECT_SOURCE_DIR}/common/cpp/include)

include(git_version)
include(ftl_paths)

if (WIN32) # TODO(nick) Should do based upon compiler (VS)
	add_definitions(-DWIN32)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++17")
	set(CMAKE_CXX_FLAGS_DEBUG "-DFTL_DEBUG -Wall")
	set(CMAKE_CXX_FLAGS_RELEASE "/O2")
else()
	add_definitions(-DUNIX)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -msse3")
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -pg -Wall")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mfpmath=sse")
endif()

SET(CMAKE_USE_RELATIVE_PATHS ON)

add_subdirectory(common/cpp)
add_subdirectory(net)

if (BUILD_RENDERER)
	add_subdirectory(renderer)
	set(HAVE_RENDER)
endif()

if (BUILD_VISION)
	add_subdirectory(vision)
endif()

if (BUILD_RECONSTRUCT)
	add_subdirectory(reconstruct)
endif()

### Generate Build Configuration Files =========================================

configure_file(${CMAKE_SOURCE_DIR}/common/cpp/include/ftl/config.h.in
               ${CMAKE_SOURCE_DIR}/common/cpp/include/ftl/config.h
)

configure_file(${CMAKE_SOURCE_DIR}/common/cpp/src/config.cpp.in
               ${CMAKE_SOURCE_DIR}/common/cpp/src/config.cpp
)

# For issue #17
# https://gitlab.kitware.com/cmake/cmake/issues/16915#note_456382
if ( TARGET Qt5::Core )
	get_property( core_options TARGET Qt5::Core PROPERTY INTERFACE_COMPILE_OPTIONS )
	string( REPLACE "-fPIC" "" new_core_options "${core_options}" )
	set_property( TARGET Qt5::Core PROPERTY INTERFACE_COMPILE_OPTIONS ${new_core_options} )
	set_property( TARGET Qt5::Core PROPERTY INTERFACE_POSITION_INDEPENDENT_CODE "ON" )
	set( CMAKE_CXX_COMPILE_OPTIONS_PIE "-fPIC" )
endif()

