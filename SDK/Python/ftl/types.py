from typing import NamedTuple
from enum import IntEnum

class Pipeline(IntEnum):
	DEPTH = 0
	RECONSTRUCT = 1

# components/rgbd-sources/include/ftl/rgbd/camera.hpp
class Camera(NamedTuple):
    fx : float
    fy : float
    cx : float
    cy : float
    width : int
    height : int
    min_depth : float
    max_depth : float
    baseline : float
    doff : float

# components/codecs/include/ftl/codecs/channels.hpp
class Channel(IntEnum):
    None_           = -1
    Colour			= 0	# 8UC3 or 8UC4
    Left			= 0
    Depth			= 1	# 32S or 32F
    Right			= 2	# 8UC3 or 8UC4
    Colour2			= 2
    Depth2			= 3
    Deviation		= 4
    Screen			= 4
    Normals			= 5	# 16FC4
    Weights			= 6	# short
    Confidence		= 7	# 32F
    Contribution	= 7	# 32F
    EnergyVector	= 8	# 32FC4
    Flow			= 9	# 32F
    Energy			= 10	# 32F
    Mask			= 11	# 32U
    Density			= 12	# 32F
    Support1		= 13	# 8UC4 (currently)
    Support2		= 14	# 8UC4 (currently)
    Segmentation	= 15	# 32S?
    Normals2		= 16	# 16FC4
    ColourHighRes	= 17	# 8UC3 or 8UC4
    LeftHighRes		= 17	# 8UC3 or 8UC4
    Disparity		= 18
    Smoothing		= 19	# 32F
    RightHighRes	= 20	# 8UC3 or 8UC4
    Colour2HighRes	= 20
    Overlay			= 21   # 8UC4
    GroundTruth		= 22	# 32F

    Audio			= 32
    AudioMono		= 32
    AudioStereo		= 33

    Configuration	= 64	# JSON Data
    Settings1		= 65
    Calibration		= 65	# Camera Parameters Object
    Pose			= 66	# Eigen::Matrix4d
    Settings2		= 67
    Calibration2	= 67	# Right camera parameters
    Index           = 68
    Control			= 69	# For stream and encoder control
    Settings3		= 70

    Data			= 2048	# Custom data any codec.
    Faces			= 2049 # Data about detected faces
    Transforms		= 2050	# Transformation matrices for framesets
    Shapes3D		= 2051	# Labeled 3D shapes
    Messages		= 2052	# Vector of Strings

_float_channels = [
    Channel.Depth,
    Channel.Confidence,
    Channel.Density,
    Channel.Energy,
    Channel.GroundTruth,
	Channel.Flow
]

def is_float_channel(channel):
    return channel in _float_channels
