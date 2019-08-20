# Future Tech Lab

This monorepo contains all elements of the FTL software system.

* [Components](components/) : Modular components compiled as static libs
  * [net](components/net/) : A p2p messaging library for C++ and JavaScript
  * [rgbd-sources](components/rgbd-sources/) : Abstraction and implementations of different RGB-Depth data sources
  * renderers : A collection of visualisation tools, including for RGB-D and point clouds
  * scene-sources : Abstraction and implementations of 3D scene data sources
  * [common](components/common/) : Utility and configuration tools
* [Applications](applications/) : Executable apps for the different node machines
  * [ftl-vision](applications/vision/) : Stereo vision node in p2p network, generates an RGB-Depth net stream
  * [ftl-reconstruct](applications/reconstruct/) : Performs scene reconstruction from synchronised RGB-Depth sources
  * calibration-multi : All camera intrinsic and extrinsic calibration in one process
  * groupview : A quick camera viewing app that supports frame and video capture
  * [ftl-gui](applications/gui/) : Desktop GUI
* front-end : Client side FTL code, both web and native
* web-service : A web backend service provider acting as a form of proxy
* www : FTL Website

The architecture is a mix of C++ and NodeJS largely communicating over a p2p RPC
network protocol.

Overall, initial vision machines capture video sources and generate colour and
depth images. These images are then streamed from all such sources to a single
machine to reconstruct the scene in 3D. The reconstructed scene is then further
compressed and streamed to other machines for additional analysis, processing or
visualisation.

The whole system is configured using JSON config files, examples of which can be
found in the `config` folder. The intention is that a web interface will
eventually enable this configuration to take place online.

## Build
Use the following commands in the root repository directory to build:

```bash
mkdir build
cd build
cmake ..
make
```

In Windows have Visual Studio installed and use the cmake graphical interface
to configure the project.

You will need to have OpenCV and glog installed. CUDA and LibSGM are optional
but recommended also. OpenCV should have cuda stereo modules compiled, along
with the viz module if local point cloud display is required. These are contrib
modules. PCL is also required to build the reconstruction components and app.

