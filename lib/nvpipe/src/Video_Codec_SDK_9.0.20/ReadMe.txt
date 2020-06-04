NVIDIA Video Codec SDK 9.0 Readme and Getting Started Guide

System Requirements

* NVIDIA Kepler/Maxwell/Pascal/Volta/Turing GPU with hardware video accelerators
  Refer to the NVIDIA Video SDK developer zone web page 
  (https://developer.nvidia.com/nvidia-video-codec-sdk) for GPUs which support 
  video encoding and decoding acceleration.

* Windows: Driver version 418.81  or higher
* Linux:   Driver version 418.30  or higher
* CUDA 10.0 Toolkit 
* NVIDIA Video Codec SDK is now supported on IBM Power9 class server with
  NVIDIA Tesla V100 (SXM2) GPU.

[Windows Configuration Requirements]
- DirectX SDK is needed. You can download the latest SDK from Microsoft's DirectX 
  website.
- The CUDA 10.0 tool kit is needed to compile the decode samples in SDK 9.0
  and above.
- CUDA tool kit is also used for building CUDA kernels that can interop with 
  NVENC.

In Windows, the following environment variables must be set to build the sample
applications included with the SDK
  - DXSDK_DIR: pointing to the DirectX SDK root directory. 
  - The CUDA 10.0 Toolkit is optional to install if the client has 
    Video Codec SDK 8.0. However it is mandatory if client has 
    Video Codec SDK 8.1 or above on his/her machine.

[Linux Configuration Requirements]    
  - X11 and OpenGL, GLUT, GLEW libraries for video playback and display 
  - The CUDA 10.0 Toolkit is optional to install if the client has Video Codec 
    SDK 8.0. 
  - CUDA 10.0 Toolkit is mandatory if client has Video Codec SDK 8.1 or above 
    on his/her machine. 
  - CUDA toolkit is used for building CUDA kernels that can interop with NVENC.
  - Libraries and headers from the FFmpeg project which can be downloaded and 
    installed using the distribution's package manager or compiled from source.
    The sample applications have been compiled and tested against the 
    libraries and headers from FFmpeg- 4.1. The source code of FFmpeg- 4.1 
    has been included in this SDK package. While configuring FFmpeg on Linux,
    it is recommended not to use 'disable-decoders' option. This configuration
    is known to have a channel error (XID 31) while executing sample
    applications with certain clips and/or result in an unexpected behavior.
  - To build/use sample applications that depend on FFmpeg, users may need to
      * Add the directory (/usr/local/lib/pkgconfig by default) to the 
        PKG_CONFIG_PATH environment variable. This is required by the Makefile
        to determine the include paths for the FFmpeg headers.
      * Add the directory where the FFmpeg libraries are installed, to the 
        LD_LIBRARY_PATH environment variable. This is required for resolving 
        runtime dependencies on FFmpeg libraries.
  - Stub libraries (libnvcuvid.so and libnvidia-encode.so) have been included
    as part of the SDK package, in order to aid development of applications on
    systems where the NVIDIA driver has not been installed. The sample
    applications in the SDK will link against these stub libraries as part of
    the build process. However, users need to ensure that the stub libraries
    are not referenced when running the sample applications. A driver
    compatible with this SDK needs to be installed in order for the sample
    applications to work correctly.
  - The Vulkan SDK needs to be installed in order to build and run the
    AppMotionEstimationVkCuda sample application. Vulkan SDK can be downloaded
    from https://vulkan.lunarg.com/sdk/home. Alternatively, it can be
    installed by using the distribution's package manager.

[Common to all OS platforms]
* To download the CUDA 10.0 toolkit, please go to the following web site:
  http://developer.nvidia.com/cuda/cuda-toolkit