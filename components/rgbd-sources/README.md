# RGB-Depth Sources

This component provides a variety of sources for colour and depth images. These
include the following:
* Intel Realsense depth camera
* Stereo video from two video capture cards using a disparity algorithm
* Snapshots that were previously captured and saved to disk
* Middlebury test datasets as available online
* Streamed network sources from other nodes

An RGB-D source is represented as a two image channel object that is generated
through a pipeline of processes usually consisting of the following (but not
all sources have all steps):
1. Frame capture from hardware
2. Internal buffer swapping if double-buffering is used
3. Retrieval, an IO blocking process of downloading images from devices
4. Computation of, for example, disparity and depth maps from colour images

## Groups
A collection of sources may form a group that must be synchronised accurately
for reconstruction to take place. A group class coordinates the above 4 steps
across all sources such that millisecond accurate frames with timestamps can
be buffered and collected together to be passed on to the next stage. A high
precision timer is used to manage the pipeline.

## Streaming
One possible use for a group of sources is to stream them over a network
where they may be re-grouped. A streamer object will receive sets of frames
from a group object and then divide each image into a number of chunks, each
of which is compressed on a CPU core and sent to every client who asks for them.
Each client may ask for a different bitrate and resolution so the streamer will
also take care of this.

## Calibration
Some sources require a camera calibration step. Lens corrections an stereo
camera configurations are applied by the calibrate class. Only stereo video
sources currently need this step. There is also some basic colour correction
that can be applied.

## Disparity Algorithms
A few algorithms are included with the RGB-D sources for converting two
colour images into one colour and one depth image based upon the pixel shift
observed between the two images. LibSGM is our algorithm of choice currently.
Further pre and post filtering and smoothing steps are applied, in particular
an optical flow based temporal smoothing across a number of frames to reduce
flickering effects.