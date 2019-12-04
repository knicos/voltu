cmake_minimum_required (VERSION 3.1.0)
include (CheckIncludeFile)
include (CheckIncludeFileCXX)
include (CheckFunctionExists)
include(CheckLanguage)

project (ftl.utu.fi)

include(GNUInstallDirs)
include(CTest)
enable_testing()

option(WITH_PCL "Use PCL if available" OFF)
option(WITH_NVPIPE "Use NvPipe for compression if available" ON)
option(WITH_OPTFLOW "Use NVIDIA Optical Flow if available" OFF)
option(WITH_OPENVR "Build with OpenVR support" OFF)
option(WITH_FIXSTARS "Use Fixstars libSGM if available" ON)
option(BUILD_VISION "Enable the vision component" ON)
option(BUILD_RECONSTRUCT "Enable the reconstruction component" ON)
option(BUILD_RENDERER "Enable the renderer component" ON)
option(BUILD_CALIBRATION "Enable the calibration component" OFF)

set(THREADS_PREFER_PTHREAD_FLAG ON)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

include(Findglog)

find_package( OpenCV REQUIRED )
find_package( Threads REQUIRED )
find_package( URIParser REQUIRED )
find_package( MsgPack REQUIRED )
find_package( Eigen3 REQUIRED )

# find_package( ffmpeg )

if (WITH_OPTFLOW)
	# TODO check that cudaoptflow.hpp exists (OpenCV built with correct contrib modules)
	set(HAVE_OPTFLOW true)
endif()

find_package( LibArchive )
if (LibArchive_FOUND)
	set(HAVE_LIBARCHIVE true)
endif()

if (WITH_OPENVR)
	## OpenVR API path
	find_library(OPENVR_LIBRARIES
		NAMES
			openvr_api
	)
	set(OPENVR_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../headers)

	if (OPENVR_LIBRARIES)
		message(STATUS "Found OpenVR: ${OPENVR_LIBRARIES}")
		set(HAVE_OPENVR true)
	endif()
endif()

if (WITH_FIXSTARS)
	find_package( LibSGM )
	if (LibSGM_FOUND)
		set(HAVE_LIBSGM true)
	endif()
endif()

if(${CMAKE_VERSION} VERSION_GREATER "3.12.0") 
	cmake_policy(SET CMP0074 NEW)
endif()

if (WITH_PCL)
	find_package( PCL QUIET COMPONENTS io common visualization registration segmentation)
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

find_library( REALSENSE_LIBRARY NAMES realsense2 librealsense2)
if (REALSENSE_LIBRARY)
	set(HAVE_REALSENSE TRUE)
	add_library(realsense UNKNOWN IMPORTED)
	#set_property(TARGET nanogui PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${NANOGUI_EXTRA_INCS})
	set_property(TARGET realsense PROPERTY IMPORTED_LOCATION ${REALSENSE_LIBRARY})
    message(STATUS "Found Realsense SDK: ${REALSENSE_LIBRARY}")
else()
	set(REALSENSE_LIBRARY "")
endif()


find_library( NANOGUI_LIBRARY NAMES nanogui libnanogui)
if (NANOGUI_LIBRARY)
	set(HAVE_NANOGUI TRUE)
	add_library(nanogui UNKNOWN IMPORTED)
	#set_property(TARGET nanogui PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${NANOGUI_EXTRA_INCS})
	set_property(TARGET nanogui PROPERTY IMPORTED_LOCATION ${NANOGUI_LIBRARY})
    message(STATUS "Found NanoGUI: ${NANOGUI_LIBRARY}")
endif()

find_library( NVPIPE_LIBRARY NAMES NvPipe libNvPipe PATHS ${NVPIPE_DIR} PATH_SUFFIXES lib)
if (NVPIPE_LIBRARY)
	set(HAVE_NVPIPE TRUE)
	add_library(nvpipe UNKNOWN IMPORTED)
	#set_property(TARGET nanogui PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${NANOGUI_EXTRA_INCS})
	set_property(TARGET nvpipe PROPERTY IMPORTED_LOCATION ${NVPIPE_LIBRARY})
	message(STATUS "Found NvPipe: ${NVPIPE_LIBRARY}")
	
	if(WIN32)
		# Find include
		find_path(NVPIPE_INCLUDE_DIRS
		    NAMES NvPipe.h
		    PATHS "C:/Program Files/NvPipe" "C:/Program Files (x86)/NvPipe" ${NVPIPE_DIR}
		    PATH_SUFFIXES include
		)
		set_property(TARGET nvpipe PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${NVPIPE_INCLUDE_DIRS})
	endif()
else()
	set(NVPIPE_LIBRARY "")
	add_library(nvpipe INTERFACE)
endif()

find_program( NODE_NPM NAMES npm )
if (NODE_NPM)
	message(STATUS "Found NPM: ${NODE_NPM}")
endif()

check_language(CUDA)
if (CUDA_TOOLKIT_ROOT_DIR)
enable_language(CUDA)
set(CMAKE_CUDA_FLAGS "")
set(CMAKE_CUDA_FLAGS_DEBUG "--gpu-architecture=compute_61 -g -DDEBUG -D_DEBUG")
set(CMAKE_CUDA_FLAGS_RELEASE "--gpu-architecture=compute_61")
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

# For ftl2mkv
check_include_file("libavformat/avformat.h" HAVE_AVFORMAT)

# Check for optional opencv components
set(CMAKE_REQUIRED_INCLUDES ${OpenCV_INCLUDE_DIRS})
check_include_file_cxx("opencv2/viz.hpp" HAVE_VIZ)
check_include_file_cxx("opencv2/cudastereo.hpp" HAVE_OPENCVCUDA)

# Optional source problem check
find_program(CPPCHECK_FOUND cppcheck)
if (CPPCHECK_FOUND)
	message(STATUS "Found cppcheck: will perform source checks")
	set(CMAKE_CXX_CPPCHECK "cppcheck" "-D__align__(A)" "-DCUDARTAPI" "--enable=warning,performance,style" "--inline-suppr" "--std=c++14" "--suppress=*:*catch.hpp" "--suppress=*:*elas*" "--suppress=*:*nanogui*" "--suppress=*:*json.hpp" "--quiet")
endif()

# include_directories(${PROJECT_SOURCE_DIR}/common/cpp/include)

include(git_version)
include(ftl_paths)

if (WIN32) # TODO(nick) Should do based upon compiler (VS)
	add_definitions(-DWIN32)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /std:c++17")
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /DFTL_DEBUG /Wall")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /O2")
	set(OS_LIBS "")
else()
	add_definitions(-DUNIX)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -msse3 -Werror=return-type")
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -D_DEBUG -pg -Wall")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -mfpmath=sse")
	set(OS_LIBS "dl")
endif()

SET(CMAKE_USE_RELATIVE_PATHS ON)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

add_subdirectory(components/common/cpp)
add_subdirectory(components/codecs)
add_subdirectory(components/net)
add_subdirectory(components/rgbd-sources)
add_subdirectory(components/control/cpp)
add_subdirectory(components/operators)
add_subdirectory(applications/calibration)
add_subdirectory(applications/groupview)
add_subdirectory(applications/player)
add_subdirectory(applications/recorder)
add_subdirectory(applications/merger)

if (HAVE_AVFORMAT)
	add_subdirectory(applications/ftl2mkv)
endif()

if (BUILD_RENDERER)
	add_subdirectory(components/renderers)
	set(HAVE_RENDER)
endif()

if (BUILD_VISION)
	add_subdirectory(applications/vision)
endif()

if (BUILD_CALIBRATION)
	find_package( cvsba REQUIRED )
	add_subdirectory(applications/calibration-multi)
endif()

if (HAVE_PCL)
	#add_subdirectory(applications/registration)
endif()

if (BUILD_RECONSTRUCT)
	add_subdirectory(applications/reconstruct)
endif()

if (HAVE_NANOGUI)
	add_subdirectory(applications/gui)
endif()

### Generate Build Configuration Files =========================================

configure_file(${CMAKE_SOURCE_DIR}/components/common/cpp/include/ftl/config.h.in
               ${CMAKE_SOURCE_DIR}/components/common/cpp/include/ftl/config.h
)

configure_file(${CMAKE_SOURCE_DIR}/components/common/cpp/src/config.cpp.in
               ${CMAKE_SOURCE_DIR}/components/common/cpp/src/config.cpp
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

if (WIN32) # TODO(nick) Should do based upon compiler (VS)
	set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${VS_STARTUP_PROJECT})
	set_property(TARGET ftl-vision PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${VS_DEBUG_WORKING_DIRECTORY})
endif()