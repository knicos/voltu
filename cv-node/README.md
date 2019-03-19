# Computer Vision Node
The program to be run on the camera compute nodes to calculate disparity from
stereo. Eventually this will also then send this data over the network to be
combined from all nodes.

## Build

`
cd <source_directory>
mkdir build
cd build
cmake ..
make
`

## Install
TODO. Currently, copy `<source_direction>/config/config.json` to
`~/.config/ftl/config.json` on Linux.

## Usage
An optional command-line argument can be passed to specify a stereo video file
to be used instead of directly attached cameras. The stereo video consists of
two videos (left and right) side by side to create a wider single video.

A `config.json` file should be located in `~/.config/ftl`.

All options found in the config file can be changed on the command line using
JSON pointers. For example: `--disparity/algorithm=\"sgbm\"` will change the
JSON config option at `{ "disparity": { "algorithm": "sgbm" }}`.

### Calibration
Cameras can be calibrated by using argument `--calibrate`. You either
need to provide a calibration video as an argument or do it live. A checkerboard
grid pattern is required, the size can be configured in the json file. After
callibration the data is save and will be reloaded automatically next time you
run `cv-node`.

Note: best calibration is not too close or far from the cameras, the board must
be fully visible in each camera and move to cover as much of the visual field
as possible.

In the terminal it prints RMS error values for the stereo calibration that need
to be around 0.3 or less ideally.

### Visualisation
The default visualisation is a 3D point cloud. however you can request a
display of the depth map, disparity map or original images.

* `--display/depth=true`
* `--display/disparity=true`
* `--display/points=false`

