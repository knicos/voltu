# Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#  * Neither the name of NVIDIA CORPORATION nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
# PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
# CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
# EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
# OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.



# Construct path to CUDA driver API lib (not provided by FindCUDA)
#get_filename_component(CUDA_LIB_DIR ${CUDA_cudart_static_LIBRARY} DIRECTORY)
#find_library(CUDA_LIB NAMES cuda HINTS ${CUDA_LIB_DIR})

# Options
option(NVPIPE_WITH_ENCODER "Enables the NvPipe encoding interface." ON)
option(NVPIPE_WITH_DECODER "Enables the NvPipe decoding interface." OFF)
option(NVPIPE_WITH_OPENGL "Enables the NvPipe OpenGL interface." OFF)
option(NVPIPE_BUILD_EXAMPLES "Builds the NvPipe example applications (requires both encoder and decoder)." OFF)

# Header
configure_file(src/NvPipe.h.in include/NvPipe.h @ONLY)
#include_directories(${CMAKE_CURRENT_BINARY_DIR}/include)

# NvPipe shared library
list(APPEND NVPIPE_SOURCES
    src/NvPipe.cu
    src/Video_Codec_SDK_9.0.20/Samples/Utils/ColorSpace.cu
    )
list(APPEND NVPIPE_LIBRARIES
	#${CUDA_LIB}
	cuda
    )

if (NVPIPE_WITH_ENCODER)
    list(APPEND NVPIPE_SOURCES
        src/Video_Codec_SDK_9.0.20/Samples/NvCodec/NvEncoder/NvEncoder.cpp
        src/Video_Codec_SDK_9.0.20/Samples/NvCodec/NvEncoder/NvEncoderCuda.cpp
        )
endif()

#if (NVPIPE_WITH_DECODER)
#    list(APPEND NVPIPE_SOURCES
#        src/Video_Codec_SDK_9.0.20/Samples/NvCodec/NvDecoder/NvDecoder.cpp
#        )
#    list(APPEND NVPIPE_LIBRARIES
#        nvcuvid
#        )

    #if (WIN32)
    #    if (CMAKE_SIZEOF_VOID_P EQUAL 8)
    #        link_directories(src/Video_Codec_SDK_9.0.20/Lib/x64)
    #    elseif (CMAKE_SIZEOF_VOID_P EQUAL 4)
    #        link_directories(src/Video_Codec_SDK_9.0.20/Lib/Win32)
    #    endif()
    #endif()
#endif()

#set(CMAKE_CUDA_COMPILE_SEPARABLE_COMPILATION ON)

add_library(NvPipe STATIC ${NVPIPE_SOURCES})
#set_property(TARGET NvPipe PROPERTY CUDA_SEPARABLE_COMPILATION ON)
#set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(NvPipe PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<INSTALL_INTERFACE:$<INSTALL_PREFIX>/include>
    )
target_include_directories(NvPipe PRIVATE
    $<BUILD_INTERFACE:src/Video_Codec_SDK_9.0.20/Samples src/Video_Codec_SDK_9.0.20/Samples/NvCodec src/Video_Codec_SDK_9.0.20/include ${CUDA_INCLUDE_DIRS}>
    )
target_link_libraries(NvPipe ${NVPIPE_LIBRARIES})

set_target_properties(NvPipe PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1)
