import msgpack

import numpy as np

from enum import IntEnum
from collections import namedtuple
from . libde265 import Decoder

try:
    import cv2 as cv
    
    def _ycrcb2rgb(img):
        return cv.cvtColor(img, cv.COLOR_YCrCb2RGB)
    
except ImportError:
    def _ycrcb2rgb(img):
        ''' YCrCb to RGB, based on OpenCV documentation definition.
        
        Note: It seems this implementation is not perfectly equivalent to OpenCV's
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

_packet = namedtuple("Packet", ["codec", "definition", "block_total", "block_number", "flags", "data"])
_stream_packet = namedtuple("StreamPacket", ["timestamp", "streamID", "chanel_count", "channel"])

_definition_t = {
    0 : (),
    1 : (),
    2 : (1080, 1920),
    3 : (720, 1280),
    4 : (),
    5 : (),
    6 : (),
    7 : (),
    8 : ()
}

class NALType(IntEnum):
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
    
    return NALType((data[4] >> 1) & 0x3f)

class FTLStream:
    def __init__(self, file):
        self._file = open(file, "br")
        self._decoders = {}
        self._frames = {}
        
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
        ''' Update calibration '''
        pass
    
    def _update_pose(self, sp, p):
        ''' Update pose '''
        pass
    
    def _decode_frame_hevc(self, sp, p):
        ''' Decode HEVC frame '''
        
        k = (sp.streamID, sp.channel)
        
        if k not in self._decoders:
            self._decoders[k] = Decoder(_definition_t[p.definition])
        
        decoder = self._decoders[k]
        
        decoder.push_data(p.data)
        decoder.decode()
        
        img = decoder.get_next_picture()
        
        if img is not None:
            self._frames[k] = _ycrcb2rgb(img)
    
    def _flush_decoders(self):
        for decoder in self._decoders.values():
            decoder.flush_data()
    
    def read(self):
        '''
        Reads data for until the next timestamp. Returns False if there is no
        more data to read, otherwise returns True.
        '''
        if self._packets_read == 0:
            self._sp, self._p = self._read_next()
            self._packets_read += 1
            
        self._frames = {}
        
        ts = self._sp.timestamp
        ex = None
        
        while self._sp.timestamp == ts:
            try:
                if self._p.codec == 100: # JSON
                    NotImplementedError("json decoding not implemented")

                elif self._p.codec == 101: # CALIBRATION
                    self._update_calib(self._sp, self._p)

                elif self._p.codec == 102: # POSE
                    self._update_pose(self._sp, self._p)

                elif self._p.codec == 3: # HEVC
                    self._decode_frame_hevc(self._sp, self._p)

                else:
                    raise ValueError("unkowno codec %i" % p.codec)
            
            except Exception as e:
                # TODO: Multiple exceptions possible. Re-design read()?
                ex = e
            
            try:
                self._sp, self._p = self._read_next()
                self._packets_read += 1
            
            except msgpack.OutOfData:
                return False
            
        if ex is not None:
            raise ex
            
        return True
    
    def get_frames(self):
        ''' Returns all frames '''
        return self._frames
    
    def get_frame(self, source, channel):
        k = (source, channel)
        if k in self._frames:
            return self._frames[k]
        else:
            return None
