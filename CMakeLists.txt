cmake_minimum_required (VERSION 3.16.0)
include (CheckIncludeFile)
include (CheckIncludeFileCXX)
include (CheckFunctionExists)
include(CheckLanguage)
include(ExternalProject)

if (WIN32)
	set(CMAKE_GENERATOR_TOOLSET "host=x64")
endif()

project (ftl.utu.fi VERSION 0.0.4)

include(GNUInstallDirs)
include(CTest)

enable_testing()

option(WITH_OPTFLOW "Use NVIDIA Optical Flow if available" OFF)
option(WITH_OPENVR "Build with OpenVR support" OFF)
option(WITH_OPUS "Use Opus audio compression" ON)
option(WITH_FIXSTARS "Use Fixstars libSGM" ON)
option(WITH_CERES "Use Ceres solver" ON)
option(WITH_SDK "Build the C shared SDK" ON)
option(USE_CPPCHECK "Apply cppcheck during build" ON)
option(BUILD_VISION "Enable the vision component" ON)
option(BUILD_RECONSTRUCT "Enable the reconstruction component" ON)
option(BUILD_RENDERER "Enable the renderer component" ON)
option(BUILD_GUI "Enable the GUI" ON)
option(BUILD_TOOLS "Compile developer and research tools" ON)
option(BUILD_TESTS "Compile all unit and integration tests" ON)
option(ENABLE_PROFILER "Enable builtin performance profiling" OFF)

if (BUILD_TESTS)
	include_directories(lib/catch)
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

include(Findglog)

MACRO( VERSION_STR_TO_INTS major minor patch version )

	STRING( REGEX REPLACE "([0-9]+).[0-9]+.[0-9]+" "\\1" ${major} ${version} )
	STRING( REGEX REPLACE "[0-9]+.([0-9]+).[0-9]+" "\\1" ${minor} ${version} )
	STRING( REGEX REPLACE "[0-9]+.[0-9]+.([0-9]+)" "\\1" ${patch} ${version} )

ENDMACRO( VERSION_STR_TO_INTS )

if (CMAKE_COMPILER_IS_GNUCXX)
	set(CMAKE_CUDA_HOST_COMPILER gcc-7)
endif()

find_package( OpenCV REQUIRED COMPONENTS core imgproc highgui cudaimgproc calib3d imgcodecs videoio aruco cudaarithm cudastereo cudaoptflow face tracking quality xfeatures2d)
find_package( Threads REQUIRED )
find_package( URIParser REQUIRED )
find_package( MsgPack REQUIRED )
find_package( Eigen3 REQUIRED )

find_package( Pylon )

VERSION_STR_TO_INTS(OPENCV_MAJOR OPENCV_MINOR OPENCV_PATCH ${OpenCV_VERSION})
math(EXPR OPENCV_NUMBER "(${OPENCV_MAJOR} * 10000) + (${OPENCV_MINOR} * 100) + ${OPENCV_PATCH}")

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
	set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
	message(STATUS "Found ccache: ${CCACHE_PROGRAM}")
endif()

# find_package( ffmpeg )

if (WITH_OPTFLOW)
	# TODO check that cudaoptflow.hpp exists (OpenCV built with correct contrib modules)
	set(HAVE_OPTFLOW true)
endif()

# Enable if the old snapshot is still needed
#find_package( LibArchive )
#if (LibArchive_FOUND)
#	set(HAVE_LIBARCHIVE true)
#endif()

#if (WITH_OPENVR)
	## OpenVR API path
#	find_library(OPENVR_LIBRARIES
#		NAMES
#			openvr_api
#	)
#	set(OPENVR_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../headers)
#
#	if (OPENVR_LIBRARIES)
#		message(STATUS "Found OpenVR: ${OPENVR_LIBRARIES}")
#		set(HAVE_OPENVR true)
#	endif()
#endif()

# ============== OPEN VR =======================================================

if (WITH_OPENVR)
	find_library( OPENVR_LIBRARY NAMES openvr_api libopenvr_api openvr_api64 PATHS ${OPENVR_DIR} PATH_SUFFIXES lib)
	if (OPENVR_LIBRARY)
		set(HAVE_OPENVR TRUE)
		add_library(openvr UNKNOWN IMPORTED)
		#set_property(TARGET nanogui PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${NANOGUI_EXTRA_INCS})
		set_property(TARGET openvr PROPERTY IMPORTED_LOCATION ${OPENVR_LIBRARY})
		message(STATUS "Found OpenVR: ${OPENVR_LIBRARY}")

		if(WIN32)
			# Find include
			find_path(OPENVR_INCLUDE_DIRS
				NAMES openvr/openvr.h
				PATHS "C:/Program Files/OpenVRSDK" "C:/Program Files (x86)/OpenVRSDK" ${OPENVR_DIR}
				PATH_SUFFIXES include
			)
			set_property(TARGET openvr PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${OPENVR_INCLUDE_DIRS})
		endif()
	else()
		set(OPENVR_LIBRARY "")
		add_library(openvr INTERFACE)
	endif()
else()
	set(OPENVR_LIBRARY "")
	add_library(openvr INTERFACE)
endif()

# ============== Opus ==========================================================

if (WITH_OPUS)
	find_library( OPUS_LIBRARY NAMES opus PATHS ${OPUS_DIR} PATH_SUFFIXES lib)

	if (OPUS_LIBRARY)
		find_path( OPUS_INCLUDE NAMES opus/opus.h)

		if (WIN32 OR OPUS_INCLUDE)
			set(HAVE_OPUS TRUE)
			add_library(Opus UNKNOWN IMPORTED)
			#set_property(TARGET nanogui PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${NANOGUI_EXTRA_INCS})
			set_property(TARGET Opus PROPERTY IMPORTED_LOCATION ${OPUS_LIBRARY})
			message(STATUS "Found Opus: ${OPUS_LIBRARY}")
		else()
			message(STATUS "Opus headers not installed")
		endif()

		if(WIN32)
			# Find include
			find_path(OPUS_INCLUDE_DIRS
				NAMES opus/opus.h
				PATHS "C:/Program Files/Opus" "C:/Program Files (x86)/Opus" ${OPUS_DIR}
				PATH_SUFFIXES include
			)
			set_property(TARGET Opus PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${OPUS_INCLUDE_DIRS})
		endif()
	else()
		message(STATUS "No Opus, audio compression disabled")
		set(OPUS_LIBRARY "")
		add_library(Opus INTERFACE)
	endif()
else()
	set(OPUS_LIBRARY "")
	add_library(Opus INTERFACE)
endif()

# ==============================================================================

add_subdirectory(lib/libstereo)
include_directories(lib/libstereo/include)
set_property(TARGET libstereo PROPERTY FOLDER "dependencies")

# ==== Ceres ===================================================================

if (WITH_CERES)
	#find_package(glog QUIET)
	find_package(Ceres REQUIRED)
	set(HAVE_CERES true)

	if (WIN32)
		# Hack to fix missing includes
		set_property(TARGET ceres PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${Ceres_DIR}/../include)
	endif()
else()
	add_library(ceres INTERFACE)
endif()

# ==============================================================================

if(${CMAKE_VERSION} VERSION_GREATER "3.12.0")
	cmake_policy(SET CMP0074 NEW)
endif()

set(CMAKE_CXX_STANDARD 17)
set(HAVE_OPENCV TRUE)

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

# ==== Realsense ===============================================================

find_library( REALSENSE_LIBRARY NAMES realsense2 librealsense2 PATHS ${REALSENSE_DIR} PATH_SUFFIXES lib/x64)
if (REALSENSE_LIBRARY)
	set(HAVE_REALSENSE TRUE)
	add_library(realsense UNKNOWN IMPORTED)
	#set_property(TARGET nanogui PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${NANOGUI_EXTRA_INCS})
	set_property(TARGET realsense PROPERTY IMPORTED_LOCATION ${REALSENSE_LIBRARY})
	message(STATUS "Found Realsense SDK: ${REALSENSE_LIBRARY}")

	if(WIN32)
		# Find include
		find_path(REALSENSE_INCLUDE_DIRS
			NAMES librealsense2/rs.hpp
			PATHS "C:/Program Files/Intel RealSense SDK 2.0" "C:/Program Files (x86)/Intel RealSense SDK 2.0" ${REALSENSE_DIR}
			PATH_SUFFIXES include
		)
		set_property(TARGET realsense PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${REALSENSE_INCLUDE_DIRS})
	endif()
else()
	set(REALSENSE_LIBRARY "")
	add_library(realsense INTERFACE)
endif()

# ==== Portaudio v19 ===========================================================

# Portaudio v19 library
find_library( PORTAUDIO_LIBRARY NAMES portaudio PATHS ${PORTAUDIO_DIR} PATH_SUFFIXES lib)
if (PORTAUDIO_LIBRARY)
	set(HAVE_PORTAUDIO TRUE)
	add_library(portaudio UNKNOWN IMPORTED)
	#set_property(TARGET nanogui PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${NANOGUI_EXTRA_INCS})
	set_property(TARGET portaudio PROPERTY IMPORTED_LOCATION ${PORTAUDIO_LIBRARY})
	message(STATUS "Found Portaudio: ${PORTAUDIO_LIBRARY}")

	if(WIN32)
		# Find include
		find_path(PORTAUDIO_INCLUDE_DIRS
			NAMES portaudio.h
			PATHS "C:/Program Files/Portaudio" "C:/Program Files (x86)/Portaudio" ${PORTAUDIO_DIR}
			PATH_SUFFIXES include
		)
		set_property(TARGET portaudio PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${PORTAUDIO_INCLUDE_DIRS})
	endif()
else()
	set(PORTAUDIO_LIBRARY "")
	add_library(portaudio INTERFACE)
	message(WARNING "Portaudio not found - sound disabled")
endif()

# ==============================================================================

# Assimp library
#find_library( ASSIMP_LIBRARY NAMES assimp PATHS ${PORTAUDIO_DIR} PATH_SUFFIXES lib)
#if (ASSIMP_LIBRARY)
#	set(HAVE_ASSIMP TRUE)
#	add_library(assimp UNKNOWN IMPORTED)
	#set_property(TARGET nanogui PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${NANOGUI_EXTRA_INCS})
#	set_property(TARGET assimp PROPERTY IMPORTED_LOCATION ${ASSIMP_LIBRARY})
#	message(STATUS "Found Assimp: ${ASSIMP_LIBRARY}")

#	if(WIN32)
		# Find include
#		find_path(ASSIMP_INCLUDE_DIRS
#			NAMES assimp/scene.h
#			PATHS "C:/Program Files/Assimp" "C:/Program Files (x86)/Assimp" ${ASSIMP_DIR}
#			PATH_SUFFIXES include
#		)
#		set_property(TARGET assimp PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${ASSIMP_INCLUDE_DIRS})
#	endif()
#else()
#	set(ASSIMP_LIBRARY "")
#	add_library(assimp INTERFACE)
#	message(WARNING "Assimp not found - no model rendering")
#endif()

find_program( NODE_NPM NAMES npm )
if (NODE_NPM)
	message(STATUS "Found NPM: ${NODE_NPM}")
endif()

check_language(CUDA)
if (CUDA_TOOLKIT_ROOT_DIR)
enable_language(CUDA)

if (NOT WIN32)
	set(CMAKE_CUDA_FLAGS "-Xcompiler -fPIC")
endif()
set(CMAKE_CUDA_ARCHITECTURES 61)
set(CMAKE_CUDA_FLAGS_DEBUG "--gpu-architecture=compute_61 -g -DDEBUG -D_DEBUG")
set(CMAKE_CUDA_FLAGS_RELEASE "--gpu-architecture=compute_61")
set(HAVE_CUDA TRUE)
include_directories(${CMAKE_CUDA_TOOLKIT_INCLUDE_DIRECTORIES})

# Some kind of fix for nvcc and -pthread problem on Linux
if (NOT WIN32)
set_property(TARGET Threads::Threads
				 PROPERTY INTERFACE_COMPILE_OPTIONS $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler -pthread>
													"$<$<NOT:$<COMPILE_LANGUAGE:CUDA>>:-pthread>")
endif()

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

# For screen capture
if (WIN32)
	message(STATUS "Screen capture not supported")
else()
	find_package(X11)
	if (X11_FOUND)
		if (X11_XShm_FOUND)
			message(STATUS "Using X11 for screen capture")
			set(HAVE_X11 TRUE)
		else()
			message(STATUS "No X11 Shared memory extension")
		endif()
	else()
		message(STATUS "No X11, screen capture disabled")
	endif()
endif()

# For ftl2mkv
check_include_file("libavformat/avformat.h" HAVE_AVFORMAT)

# Check for optional opencv components
set(CMAKE_REQUIRED_INCLUDES ${OpenCV_INCLUDE_DIRS})
check_include_file_cxx("opencv2/viz.hpp" HAVE_VIZ)
check_include_file_cxx("opencv2/cudastereo.hpp" HAVE_OPENCVCUDA)

# Optional source problem check
if (USE_CPPCHECK)
	find_program(CPPCHECK_FOUND cppcheck)
	if (CPPCHECK_FOUND)
		message(STATUS "Found cppcheck: will perform source checks")
		set(CMAKE_CXX_CPPCHECK "cppcheck" "-D__align__(A)" "-DCUDARTAPI" "--enable=warning,performance,style" "--inline-suppr" "--std=c++14" "--suppress=*:*catch.hpp" "--suppress=*:*elas*" "--suppress=*:*nanogui*" "--suppress=*:*json.hpp" "--quiet")
	endif()
endif()

# include_directories(${PROJECT_SOURCE_DIR}/common/cpp/include)

include(git_version)
include(ftl_paths)

if (WIN32) # TODO(nick) Should do based upon compiler (VS)
	add_definitions(-DWIN32)
	set(CMAKE_GENERATOR_TOOLSET "host=x64")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /arch:AVX2 /MP4 /std:c++17 /wd4996")
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /DFTL_DEBUG /Wall")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /O2")
	set(OS_LIBS "")
else()
	add_definitions(-DUNIX)
	# -fdiagnostics-color
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -fPIC -march=native -mfpmath=sse -Wall -Werror=unused-result -Werror=return-type")
	set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -pg")
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")
	set(OS_LIBS "dl")
endif()

SET(CMAKE_USE_RELATIVE_PATHS ON)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# ==== NvPipe ==================================================================

#add_subdirectory(lib/nvpipe)

#find_library( NVPIPE_LIBRARY NAMES NvPipe libNvPipe PATHS ${NVPIPE_DIR} PATH_SUFFIXES lib)
#if (NVPIPE_LIBRARY)
	set(HAVE_NVPIPE TRUE)
#	add_library(nvpipe UNKNOWN IMPORTED)
	#set_property(TARGET nanogui PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${NANOGUI_EXTRA_INCS})
#	set_property(TARGET nvpipe PROPERTY IMPORTED_LOCATION ${NVPIPE_LIBRARY})
#	message(STATUS "Found NvPipe: ${NVPIPE_LIBRARY}")

#	if(WIN32)
		# Find include
#		find_path(NVPIPE_INCLUDE_DIRS
#			NAMES NvPipe.h
#			PATHS "C:/Program Files/NvPipe" "C:/Program Files (x86)/NvPipe" ${NVPIPE_DIR}
#			PATH_SUFFIXES include
#		)
#		set_property(TARGET nvpipe PROPERTY INTERFACE_INCLUDE_DIRECTORIES ${NVPIPE_INCLUDE_DIRS})
#	endif()
#else()
#	set(NVPIPE_LIBRARY "")
#	add_library(nvpipe INTERFACE)
#endif()

if (WIN32)
	add_library(nvidia-ml INTERFACE)
endif()

if (WITH_FIXSTARS)
	set(HAVE_LIBSGM true)
	add_subdirectory(lib/libsgm)
	include_directories(lib/libsgm/include)
	set_property(TARGET sgm PROPERTY FOLDER "dependencies")
else()
	add_library(sgm INTERFACE)
endif()

# ==============================================================================

if (BUILD_GUI)
	set(HAVE_NANOGUI TRUE)

	# Disable building extras we won't need (pure C++ project)
	set(NANOGUI_BUILD_SHARED OFF CACHE BOOL " " FORCE)
	set(NANOGUI_BUILD_EXAMPLE OFF CACHE BOOL " " FORCE)
	set(NANOGUI_BUILD_PYTHON  OFF CACHE BOOL " " FORCE)
	set(NANOGUI_INSTALL       OFF CACHE BOOL " " FORCE)
	set(NANOGUI_EIGEN_INCLUDE_DIR ${EIGEN_INCLUDE_DIR} CACHE STRING " " FORCE)

	# Add the configurations from nanogui
	add_subdirectory(ext/nanogui)

	# For reliability of parallel build, make the NanoGUI targets dependencies
	set_property(TARGET glfw glfw_objects nanogui PROPERTY FOLDER "dependencies")
endif()

# =============================================================================


add_subdirectory(lib/cudatl)
add_subdirectory(components/common/cpp)
add_subdirectory(components/codecs)
add_subdirectory(components/structures)
add_subdirectory(components/net)
add_subdirectory(components/rgbd-sources)
add_subdirectory(components/control/cpp)
add_subdirectory(components/operators)
add_subdirectory(components/streams)
add_subdirectory(components/audio)
add_subdirectory(components/calibration)
add_subdirectory(components/disparity)
#add_subdirectory(applications/groupview)
#add_subdirectory(applications/player)
#add_subdirectory(applications/recorder)
#add_subdirectory(applications/merger)
add_subdirectory(applications/tools)

# SDK only compiles on linux currently
if (WITH_SDK)
	if (NOT WIN32)
		add_subdirectory(SDK/C)
	endif()
	add_subdirectory(SDK/CPP)
endif()

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

if (BUILD_RECONSTRUCT)
	add_subdirectory(applications/reconstruct2)
endif()

if (HAVE_NANOGUI)
	#add_subdirectory(applications/gui)
	add_subdirectory(applications/gui2)
endif()

### Generate Build Configuration Files =========================================

configure_file(${CMAKE_SOURCE_DIR}/components/common/cpp/include/ftl/config.h.in
			   ${CMAKE_SOURCE_DIR}/components/common/cpp/include/ftl/config.h
)

configure_file(${CMAKE_SOURCE_DIR}/components/common/cpp/src/config.cpp.in
			   ${CMAKE_SOURCE_DIR}/components/common/cpp/src/config.cpp
)

if (WIN32) # TODO(nick) Should do based upon compiler (VS)
	set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${VS_STARTUP_PROJECT})
	set_property(TARGET voltu-vision PROPERTY VS_DEBUGGER_WORKING_DIRECTORY ${VS_DEBUG_WORKING_DIRECTORY})
endif()

include(ftl_CPack)
