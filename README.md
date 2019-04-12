# Future Tech Lab

This monorepo contains all elements of the FTL software system.

* net : A p2p messaging library for C++ and JavaScript
* vision : Stereo vision node in p2p network
* reconstruct : Performs scene reconstruction from vision nodes
* renderer : Produce video or live feeds from scene representation
* front-end : Client side FTL code, both web and native
* web-service : A web backend service provider acting as a form of proxy
* www : FTL Website

The architecture is a mix of C++ and NodeJS largely communicating over a p2p
network protocol.

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
modules.

