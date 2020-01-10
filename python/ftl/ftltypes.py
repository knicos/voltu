
from collections import namedtuple
from enum import IntEnum

# components/rgbd-sources/include/ftl/rgbd/camera.hpp
Camera = namedtuple("Camera", ["fx", "fy", "cx", "cy", "width", "height",
                                "min_depth", "max_depth", "baseline", "doff"])

# components/codecs/include/ftl/codecs/packet.hpp
Packet = namedtuple("Packet", ["codec", "definition", "block_total",
                               "block_number", "flags", "data"])

StreamPacket = namedtuple("StreamPacket", ["timestamp", "streamID",
                                           "channel_count", "channel"])

class PacketFlags:
    RGB = 0x00000001
    MappedDepth = 0x00000002

# components/codecs/include/ftl/codecs/channels.hpp
class Channel(IntEnum):
    None_           = -1
    Colour          = 0
    Left            = 0
    Depth           = 1
    Right           = 2
    Colour2         = 2
    Disparity       = 3
    Depth2          = 3
    Deviation       = 4
    Normals         = 5
    Points          = 6
    Confidence      = 7
    Contribution    = 7
    EnergyVector    = 8
    Flow            = 9
    Energy          = 10
    Mask            = 11
    Density         = 12
    LeftGray        = 13
    RightGray       = 14
    Overlay1        = 15

    AudioLeft       = 32
    AudioRight      = 33

    Configuration   = 64
    Calibration     = 65
    Pose            = 66
    Data            = 67

_float_channels = [
    Channel.Depth,
    Channel.Confidence,
    Channel.Density,
    Channel.Energy
]

def is_float_channel(channel):
    return channel in _float_channels

# components/codecs/include/ftl/codecs/bitrates.hpp
class codec_t(IntEnum):
    JPG = 0
    PNG = 1
    H264 = 2
    HEVC = 3
    WAV = 4
    JSON = 100
    CALIBRATION = 101
    POSE = 102
    MSGPACK = 103,
    STRING = 104,
    RAW = 105

definition_t = {
    0 : (7680, 4320),
    1 : (2160, 3840),
    2 : (1080, 1920),
    3 : (720, 1280),
    4 : (576, 1024),
    5 : (480, 854),
    6 : (360, 640),
    7 : (0, 0),
    8 : (2056, 1852)
}

def get_definition(shape):
	for k, v in definition_t.items():
		if shape[:2] == v:
			return k
	
	return 7 # (None)
