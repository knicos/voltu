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
    warn("OpenCV not available. OpenCV required for full functionality.")

    def _ycrcb2rgb(img):
        ''' YCrCb to RGB, based on OpenCV documentation definition.

        Note: It seems this implementation is not perfectly equivalent to
        OpenCV's (results not exactly same, why?)
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

def _ycbcr2rgb(img):
    rgb = np.zeros(img.shape, np.float)

    Y = img[:,:,0].astype(np.float)
    Cr = img[:,:,2].astype(np.float)
    Cb = img[:,:,1].astype(np.float)
    delta = 128.0

    rgb[:,:,0] = Y + 1.403 * (Cr - delta)
    rgb[:,:,1] = Y - 0.714 * (Cr - delta) - 0.344 * (Cb - delta)
    rgb[:,:,2] = Y + 1.773 * (Cb - delta)

    return rgb.round().astype(np.uint8)

class FTLStreamWriter:
    def __init__(self, file, version=2):
        self._file = open(file, "wb")
        self._file.write(bytes(ord(c) for c in "FTLF")) # magic
        self._file.write(bytes([version]))              # version
        self._file.write(bytes([0]*64))                 # reserved

        self._packer = msgpack.Packer(strict_types=False, use_bin_type=True)

    def __del__(self):
        self.close()

    def close(self):
        self._file.close()

    def add_raw(self, sp, p):
        if len(sp) != len(ftl.StreamPacket._fields) or len(p) != len(ftl.Packet._fields):
           raise ValueError("invalid input")

        self._file.write(self._packer.pack((sp, p)))
        self._file.flush()

    def add_frame(self, timestamp, source, channel, channel_count, codec,  data,
                  definition=None, flags=0, encode=True):
        ''' Write frame to file. If encode is False (data already encoded),
        definition needs to be specified.
        '''

        if source < 0:
            raise ValueError("invalid source id")

        if channel not in ftl.Channel:
            raise ValueError("invalid channel")

        if codec not in ftl.codec_t:
            raise ValueError("invalid codec")

        if encode:
            if definition is None:
                definition = ftl.get_definition(data.shape)

            if definition is None:
                raise ValueError("unsupported resolution")

            if definition != ftl.get_definition(data.shape):
                # todo: could replace definition or scale
                raise ValueError("definition does not match frame resolution")

            if codec == ftl.codec_t.PNG:
                if ftl.is_float_channel(channel):
                    # scaling always same (???)
                    data = (data * 1000).astype(np.uint16)

                params = [cv.IMWRITE_PNG_COMPRESSION, 9]
                retval, data = cv.imencode(".png", data, params)

                if not retval:
                    raise Exception("encoding error (PNG)")

            elif codec == ftl.codec_t.JPG:
                params = []
                retval, data = cv.imencode(".jpg", data, params)

                if not retval:
                    raise Exception("encoding error (JPG)")

            else:
                raise ValueError("unsupported codec")

            data = data.tobytes()

        if definition is None:
            raise ValueError("definition required")

        if not isinstance(data, bytes):
            raise ValueError("expected bytes")

        sp = ftl.StreamPacket(int(timestamp), int(source),
                              int(channel_count), int(channel))
        p = ftl.Packet(int(codec), int(definition), 1, 0, int(flags), data)

        self.add_raw(sp, p)

    def add_pose(self, timestamp, source, data):
        if data.shape != (4, 4):
            raise ValueError("invalid pose")

        data.astype(np.float64).tobytes(order='F')
        raise NotImplementedError("todo")

    def add_calibration(self, timestamp, source, data):
        # todo: Use msgpack format instead (ftlf v3+)
        struct.pack("@ddddIIdddd", *data)
        raise NotImplementedError("todo")

class FTLStreamReader:
    ''' FTL file reader. '''

    def __init__(self, file):
        self._file = open(file, "br")
        self._version = 0

        self._decoders_hevc = {}
        self._seen_iframe = set()

        self._frame = None

        # calibration and pose are cached
        self._calibration = {}
        self._pose = {}

        try:
            magic = self._file.read(5)
            self._version = int(magic[4])
            if magic[:4] != bytes(ord(c) for c in "FTLF"):
                raise Exception("wrong magic")

            if self._version >= 2:
                # first 64 bytes reserved
                self._file.read(8*8)

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
        ''' Update calibration. '''

        if p.codec == ftl.codec_t.MSGPACK:
            # TODO: channel and capabilities should be saved as well
            calib, channel, capabilities = msgpack.unpackb(p.data)
            self._calibration[sp.streamID] = ftl.Camera._make(calib)

        elif p.codec == ftl.codec_t.CALIBRATION:
            calibration = struct.unpack("@ddddIIdddd", p.data[:(4*8+2*4+4*8)])
            self._calibration[sp.streamID] = ftl.Camera._make(calibration)

        else:
            raise Exception("Unknown codec %i for calibration" % p.codec)

    def _update_pose(self, sp, p):
        ''' Update pose '''
        pose = np.asarray(struct.unpack("@16d", p.data[:(16*8)]),
                          dtype=np.float64)
        pose = pose.reshape((4, 4), order='F') # Eigen
        self._pose[sp.streamID] = pose

    def _process_json(self, sp, p):
        raise NotImplementedError("json decoding not implemented")

    def _decode_hevc(self, sp, p):
        ''' Decode HEVC frame '''

        k = (sp.streamID, sp.channel)

        if k not in self._decoders_hevc:
            self._decoders_hevc[k] = libde265.Decoder()

        decoder = self._decoders_hevc[k]

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
            warn("frame expected, no image received from decoder")

        if ftl.is_float_channel(self._sp.channel):
            # TODO: only supports 8 bits per pixel format and 16 bits
            #       ()"old format")
            #
            # NVPipe: (2 * width), high bits in left, low in right

            high = img[:,(img.shape[1]//2):,0].astype(np.uint32) << 8
            low = img[:,:(img.shape[1]//2),0].astype(np.uint32)
            img = (high|low).astype(np.float)/1000.0

            try:
                img[img < self._calibration[sp.streamID].min_depth] = 0.0
                img[img > self._calibration[sp.streamID].max_depth] = 0.0
            except KeyError:
                warn("no calibration for received frame")

            self._frame = img

        else:
            if self._version < 3:
                self._frame = _ycrcb2rgb(img)
            else:
                self._frame = _ycbcr2rgb(img)

    def _decode_opencv(self, sp, p):
        try:
            cv
        except NameError:
            raise Exception("OpenCV required for OpenCV (png/jpeg) decoding")

        self._frame = cv.imdecode(np.frombuffer(p.data, dtype=np.uint8),
                                  cv.IMREAD_UNCHANGED)

        if ftl.is_float_channel(self._sp.channel):
            self._frame = self._frame.astype(np.float) / 1000.0

    def seek(self, ts):
        ''' Read until timestamp reached '''
        if self.get_timestamp() >= ts:
            raise Exception("trying to seek to earlier timestamp")

        while self.read():
            if self.get_timestamp() >= ts:
                break

    def read(self):
        '''
        Reads data for until the next timestamp. Returns False if there is no
        more data to read, otherwise returns True.

        todo: make (frame) decoding optional
        '''
        self._frame = None

        try:
            self._sp, self._p = self._read_next()
            self._packets_read += 1

        except msgpack.OutOfData:
            return False

        if self._p.block_total != 1 or self._p.block_number != 0:
            raise Exception("Unsupported block format (todo)")

        # calibration/pose cached
        # todo: should be done by user instead?

        if self._sp.channel == ftl.Channel.Calibration:
            self._update_calib(self._sp, self._p)

        elif self._sp.channel == ftl.Channel.Pose:
            self._update_pose(self._sp, self._p)

        # decode if codec supported
        if self._p.codec == ftl.codec_t.HEVC:
            self._decode_hevc(self._sp, self._p)

        elif self._p.codec == ftl.codec_t.PNG:
            self._decode_opencv(self._sp, self._p)

        elif self._p.codec == ftl.codec_t.JPG:
            self._decode_opencv(self._sp, self._p)

        else:
            # todo (unsupported codec)
            pass

        return True

    def get_packet_count(self):
        return self._packets_read

    def get_raw(self):
        ''' Returns previously received StreamPacket and Packet '''
        return self._sp, self._p

    def get_channel_type(self):
        return ftl.Channel(self._sp.channel)

    def get_source_id(self):
        return self._sp.streamID

    def get_timestamp(self):
        return self._sp.timestamp

    def get_frame(self):
        ''' Return decoded frame from previous packet. Returns None if previous
        packet did not contain a (valid) frame. '''
        return self._frame

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

    def get_Q(self, source):
        ''' Disparity to depth matrix (OpenCV) '''

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

    def get_version(self):
        return self._version
