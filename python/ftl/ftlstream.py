import msgpack

import numpy as np

import sys
import struct
from warnings import warn
from enum import IntEnum
from collections import namedtuple

from . misc import is_iframe
from . import ftltypes as ftl
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

class FTLStreamWriter:
    def __init__(self, file):
        self._file = open(file, "wb")
        self._file.write(bytes(ord(c) for c in "FTLF"))
        self._file.write(bytes([0]))

        self._packer = msgpack.Packer(strict_types=False, use_bin_type=True)

    def __del__(self):
        self.close()

    def close(self):
        self._file.close()

    def add_source(self, parameters, pose):
        pass

    def add_raw(self, sp, p):
        if len(sp) != len(ftl.StreamPacket._fields) or len(p) != len(ftl.Packet._fields):
           raise ValueError("invalid input")
        
        self._file.write(self._packer.pack((sp, p)))

    def add_frame(self, timestamp, src, channel, codec, data, encode=True):
        pass

    def add_depth(self, timestamp, src, data):
        pass

class FTLStreamReader:
    ''' FTL file reader '''
    
    def __init__(self, file):
        self._file = open(file, "br")
        self._decoders = {}
        self._seen_iframe = set()

        self._frameset = {}
        self._frameset_new = {}
        self._frame = None
        
        self._calibration = {}
        self._pose = {}
        self._ts = -sys.maxsize - 1

        try:
            magic = self._file.read(5)
            if magic[:4] != bytes(ord(c) for c in "FTLF"):
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
        return ftl.StreamPacket._make(v1), ftl.Packet._make(v2)
    
    def _update_calib(self, sp, p):
        ''' Update calibration.
        
        todo: fix endianess
        '''
        calibration = struct.unpack("@ddddIIdddd", p.data[:(4*8+2*4+4*8)])
        self._calibration[sp.streamID] = ftl.Camera._make(calibration)

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

    def _decode_hevc(self, sp, p):
        ''' Decode HEVC frame '''
        
        k = (sp.streamID, sp.channel)
        
        if k not in self._decoders:
            self._decoders[k] = libde265.Decoder(ftl.definition_t[p.definition])
        
        decoder = self._decoders[k]

        if k not in self._seen_iframe:
            if not is_iframe(p.data):
                # can't decode before first I-frame has been received
                warn("received P-frame before I-frame")
                return
            
            self._seen_iframe.add(k)
        
        decoder.push_data(p.data)
        decoder.push_end_of_frame()
        
        while decoder.get_number_of_input_bytes_pending() > 0:
            decoder.decode()
        
        img = decoder.get_next_picture()
        if img is None:
            # if this happens, does get_next_picture() in loop help?
            warn("frame expected, no image from decoded")
        
        self._frame = _ycrcb2rgb(img)
        self._frameset_new[k] = self._frame

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
        self._frame = None

        try:
            self._sp, self._p = self._read_next()
            self._packets_read += 1
        
        except msgpack.OutOfData:
            self._frameset = self._frameset_new
            self._frameset_new = {}
            return False
        
        if self._sp.timestamp < self._ts:
            # old data, do not update
            return True

        if self._sp.timestamp > self._ts:
            self._ts = self._sp.timestamp
            self._frameset = self._frameset_new
            self._frameset_new = {}
        
        if self._p.block_total != 1 or self._p.block_number != 0:
            raise Exception("Unsupported block format (todo)")

        if self._p.codec == ftl.codec_t.JSON:
            self._process_json(self._sp, self._p)

        elif self._p.codec == ftl.codec_t.CALIBRATION:
            self._update_calib(self._sp, self._p)

        elif self._p.codec == ftl.codec_t.POSE:
            self._update_pose(self._sp, self._p)

        elif self._p.codec == ftl.codec_t.HEVC:
            self._decode_hevc(self._sp, self._p)

        else:
            raise Exception("unkowno codec %i" % self._p.codec)

        return True

    def get_packet_count(self):
        return self._packets_read

    def get_raw(self):
        return self._sp, self._p

    def get_timestamp(self):
        return self._ts

    def get_pose(self, source):
        try:
            return self._pose[source]
        except KeyError:
            raise ValueError("source id %i not found" % source)

    def get_camera_matrix(self, source):
        ''' Camera intrinsic parameters '''

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

    def get_frame(self):
        ''' Return decoded frame from previous packet. Returns None if previous
        packet did not contain a (valid) frame. '''
        return self._frame

    def get_frameset(self):
        return self._frameset
    
    def get_frameset_frame(self, source, channel):
        k = (source, channel)
        if k in self._frameset:
            return self._frameset[k]
        else:
            # raise an exception instead?
            return None
    
    def get_frameset_sources(self):
        return list(set(src for src, _ in self._frameset.keys()))

    def get_Q(self, source):
        ''' Disparity to depth matrix in OpenCV format '''

        calib = self.get_calibration(source)
        Q = np.identity(4, dtype=np.float64)
        Q[0,3] = calib.cx
        Q[1,3] = calib.cy
        Q[2,2] = 0.0
        Q[2,3] = calib.fx
        Q[3,2] = -1 / calib.baseline
        Q[3,3] = calib.doff
        return Q

    def get_sources(self):
        ''' Get list of sources '''
        return list(self._calibration.keys())

