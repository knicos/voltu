import msgpack

import numpy as np

import struct

from enum import IntEnum
from collections import namedtuple
from . import libde265

try:
    import cv2 as cv
    
    def _ycrcb2rgb(img):
        return cv.cvtColor(img, cv.COLOR_YCrCb2RGB)
    
except ImportError:
    def _ycrcb2rgb(img):
        ''' YCrCb to RGB, based on OpenCV documentation definition.
        
        Note: It seems this implementation is not perfectly equivalent to
        OpenCV's
        '''
        
        rgb = np.zeros(img.shape, np.float)

        Y = img[:,:,0].astype(np.float)
        Cr = img[:,:,1].astype(np.float)
        Cb = img[:,:,2].astype(np.float)
        delta = 128.0

        rgb[:,:,0] = Y + 1.403 * (Cr - delta)
        rgb[:,:,1] = Y - 0.714 * (Cr - delta) - 0.344 * (Cb - delta)
        rgb[:,:,2] = Y + 1.773 * (Cb - delta)

        return rgb.round().astype(np.uint8)

# FTL definitions

# components/rgbd-sources/include/ftl/rgbd/camera.hpp
_Camera = namedtuple("Camera", ["fx", "fy", "cx", "cy", "width", "height",
                                "min_depth", "max_depth", "baseline", "doffs"])

# components/codecs/include/ftl/codecs/packet.hpp
_packet = namedtuple("Packet", ["codec", "definition", "block_total",
                                "block_number", "flags", "data"])

_stream_packet = namedtuple("StreamPacket", ["timestamp", "streamID",
                                             "chanel_count", "channel"])

# components/codecs/include/ftl/codecs/bitrates.hpp
class _codec_t(IntEnum):
    JPG = 0
    PNG = 1
    H264 = 2
    HEVC = 3
    WAV = 4
    JSON = 100
    CALIBRATION = 101
    POSE = 102
    RAW = 103

_definition_t = {
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

# components/codecs/include/ftl/codecs/hevc.hpp
class _NALType(IntEnum):
    CODED_SLICE_TRAIL_N = 0
    CODED_SLICE_TRAIL_R = 1

    CODED_SLICE_TSA_N = 2
    CODED_SLICE_TSA_R = 3

    CODED_SLICE_STSA_N = 4
    CODED_SLICE_STSA_R = 5

    CODED_SLICE_RADL_N = 6
    CODED_SLICE_RADL_R = 7

    CODED_SLICE_RASL_N = 8
    CODED_SLICE_RASL_R = 9

    RESERVED_VCL_N10 = 10
    RESERVED_VCL_R11 = 11
    RESERVED_VCL_N12 = 12
    RESERVED_VCL_R13 = 13
    RESERVED_VCL_N14 = 14
    RESERVED_VCL_R15 = 15

    CODED_SLICE_BLA_W_LP = 16
    CODED_SLICE_BLA_W_RADL = 17
    CODED_SLICE_BLA_N_LP = 18
    CODED_SLICE_IDR_W_RADL = 19
    CODED_SLICE_IDR_N_LP = 20
    CODED_SLICE_CRA = 21
    RESERVED_IRAP_VCL22 = 22
    RESERVED_IRAP_VCL23 = 23

    RESERVED_VCL24 = 24
    RESERVED_VCL25 = 25
    RESERVED_VCL26 = 26
    RESERVED_VCL27 = 27
    RESERVED_VCL28 = 28
    RESERVED_VCL29 = 29
    RESERVED_VCL30 = 30
    RESERVED_VCL31 = 31

    VPS = 32
    SPS = 33
    PPS = 34
    ACCESS_UNIT_DELIMITER = 35
    EOS = 36
    EOB = 37
    FILLER_DATA = 38
    PREFIX_SEI = 39
    SUFFIX_SEI = 40

    RESERVED_NVCL41 = 41
    RESERVED_NVCL42 = 42
    RESERVED_NVCL43 = 43
    RESERVED_NVCL44 = 44
    RESERVED_NVCL45 = 45
    RESERVED_NVCL46 = 46
    RESERVED_NVCL47 = 47
    UNSPECIFIED_48 = 48
    UNSPECIFIED_49 = 49
    UNSPECIFIED_50 = 50
    UNSPECIFIED_51 = 51
    UNSPECIFIED_52 = 52
    UNSPECIFIED_53 = 53
    UNSPECIFIED_54 = 54
    UNSPECIFIED_55 = 55
    UNSPECIFIED_56 = 56
    UNSPECIFIED_57 = 57
    UNSPECIFIED_58 = 58
    UNSPECIFIED_59 = 59
    UNSPECIFIED_60 = 60
    UNSPECIFIED_61 = 61
    UNSPECIFIED_62 = 62
    UNSPECIFIED_63 = 63
    INVALID = 64

def get_NAL_type(data):
    if not isinstance(data, bytes):
        raise ValueError("expected bytes")
    
    return _NALType((data[4] >> 1) & 0x3f)

class FTLStream:
    ''' FTL file reader '''
    
    def __init__(self, file):
        self._file = open(file, "br")
        self._decoders = {}
        self._frames = {}
        self._calibration = {}
        self._pose = {}
        self._ts = -1

        try:
            magic = self._file.read(5)
            if magic[:4] != bytearray(ord(c) for c in "FTLF"):
                raise Exception("wrong magic")
            
            self._unpacker = msgpack.Unpacker(self._file, raw=True, use_list=False)
            
        except Exception as ex:
            self._file.close()
            raise ex
            
        self._packets_read = 0
    
    def __del__(self):
        self._file.close()
    
    def _read_next(self):
        v1, v2 = self._unpacker.unpack()
        return _stream_packet._make(v1), _packet._make(v2)
    
    def _update_calib(self, sp, p):
        ''' Update calibration.
        
        todo: fix endianess
        '''
        calibration = struct.unpack("@ddddIIdddd", p.data[:(4*8+2*4+4*8)])
        self._calibration[sp.streamID] = _Camera._make(calibration)

    def _update_pose(self, sp, p):
        ''' Update pose
        
        todo: fix endianess
        '''

        pose = np.asarray(struct.unpack("@16d", p.data[:(16*8)]),
                          dtype=np.float64)
        pose = pose.reshape((4, 4), order='F') # Eigen

        self._pose[sp.streamID] = pose
    
    def _process_json(self, sp, p):
        raise NotImplementedError("json decoding not implemented")

    def _push_data_hevc(self, sp, p):
        ''' Decode HEVC frame '''
        
        k = (sp.streamID, sp.channel)
        
        if k not in self._decoders:
            self._decoders[k] = libde265.Decoder(_definition_t[p.definition])
        
        decoder = self._decoders[k]
        decoder.push_data(p.data)
        try:
            decoder.decode()

        except libde265.WaitingForInput:
            pass
    
    def _decode_hevc(self):
        for stream, decoder in self._decoders.items():
            try:
                decoder.decode()

            except libde265.WaitingForInput:
                pass

            img = decoder.get_next_picture()
            if img is not None:
                self._frames[stream] = _ycrcb2rgb(img)

    def _flush_decoders(self):
        for decoder in self._decoders.values():
            decoder.flush_data()

    def seek(self, ts):
        ''' Read until timestamp reached '''
        if self._ts >= ts:
            raise Exception("trying to seek to earlier timestamp")
        
        while self.read():
            if self._ts >= ts:
                break

    def read(self):
        '''
        Reads data for until the next timestamp. Returns False if there is no
        more data to read, otherwise returns True.
        '''
        if self._packets_read == 0:
            self._sp, self._p = self._read_next()
            self._packets_read += 1
            
        self._frames = {}
        
        self._ts = self._sp.timestamp
        ex = []
        
        while self._sp.timestamp == self._ts:
            # TODO: Can source be removed?
            
            try:
                if self._p.codec == _codec_t.JSON:
                    self._process_json(self._sp, self._p)

                elif self._p.codec == _codec_t.CALIBRATION:
                    self._update_calib(self._sp, self._p)

                elif self._p.codec == _codec_t.POSE:
                    self._update_pose(self._sp, self._p)

                elif self._p.codec == _codec_t.HEVC:
                    self._push_data_hevc(self._sp, self._p)

                else:
                    raise ValueError("unkowno codec %i" % self._p.codec)
            
            except Exception as e:
                ex.append(e)
            
            try:
                self._sp, self._p = self._read_next()
                self._packets_read += 1
            
            except msgpack.OutOfData:
                return False
            
        if len(ex) > 0:
            raise Exception(ex)
        
        self._decode_hevc()

        return True
    
    def get_timestamp(self):
        return self._ts

    def get_pose(self, source):
        try:
            return self._pose[source]
        except KeyError:
            raise ValueError("source id %i not found" % source)

    def get_camera_matrix(self, source):
        calib = self.get_calibration(source)
        K = np.identity(3, dtype=np.float64)
        K[0,0] = calib.fx
        K[1,1] = calib.fy
        K[0,2] = calib.cx
        K[1,2] = calib.cy
        return K

    def get_calibration(self, source):
        try:
            return self._calibration[source]
        except KeyError:
            raise ValueError("source id %i not found" % source)

    def get_frames(self):
        ''' Returns all frames '''
        return self._frames
    
    def get_frame(self, source, channel):
        k = (source, channel)
        if k in self._frames:
            return self._frames[k]
        else:
            # raise an exception instead?
            return None

    def get_sources(self):
        ''' Get list of sources
        
        todo: Is there a better way?
        '''
        return list(self._calibration.keys())