# RGB-Depth Sources

This component provides a variety of sources for colour and depth images. These
include the following, but do not include virtual cameras:
* [Intel Realsense depth camera](src/realsense_source.hpp)
* [Stereo video](src/stereovideo.hpp) from two video capture cards using a disparity algorithm
* [Snapshots](src/snapshot_source.hpp) that were previously captured and saved to disk
* [Middlebury](src/middlebury_source.hpp) test datasets as available online
* [Streamed network sources](include/ftl/rgbd/streamer.hpp) from other nodes

An RGB-D source is represented as a two image channel object that is generated
through a pipeline of processes usually consisting of the following (but not
all sources have all steps):
1. Frame capture from hardware
2. Internal buffer swapping if double-buffering is used
3. Retrieval, an IO blocking process of downloading images from devices
4. Computation of, for example, disparity and depth maps from colour images

## Groups
A collection of sources may form a group that must be synchronised accurately
for reconstruction to take place. A [group class](include/ftl/rgbd/group.hpp)
coordinates the above 4 steps across all sources such that millisecond accurate
frames with timestamps can be buffered and collected together to be passed on to
the next stage. A [high precision timer](../common/cpp/include/ftl/timer.hpp)
is used to manage the pipeline.

## Streaming
One possible use for a group of sources is to stream them over a network
where they may be re-grouped. A [streamer](include/ftl/rgbd/streamer.hpp) object
will receive sets of frames from a group object and then divide each image into
a number of chunks, each of which is compressed on a CPU core and sent to every
client who asks for them. Each client may ask for a different bitrate and
resolution so the streamer will also take care of this. The streamer class uses
the [ftl net library](../net/) for network communication.

## Calibration
Some sources require a camera calibration step. Lens corrections an stereo
camera configurations are applied by the [calibrate class](src/calibrate.hpp).
Only stereo video sources currently need this step and the correction matrices
are calculated using a separate
[calibration app](../../application/calibration-multi/). There is also some
basic [colour correction](src/colour.hpp) that can be applied.

## Disparity Algorithms
A few algorithms are included with the RGB-D sources for converting two
colour images into one colour and one depth image based upon the pixel shift
observed between the two images. [LibSGM](https://github.com/fixstars/libSGM)
is our algorithm of choice currently. Further pre and post filtering and
smoothing steps are applied, in particular an optical flow based temporal
smoothing across a number of frames to reduce flickering effects. This uses
[NVIDIA's Optical Flow SDK](https://developer.nvidia.com/opticalflow-sdk).