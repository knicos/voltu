import numpy as np
import msgpack

import struct
from warnings import warn

from . import ftltype
from . import libde265
from . misc import Calibration

from enum import IntEnum

_has_opencv = False
try:
    import cv2 as cv
    _has_opencv = True
except ImportError:
    warn("OpenCV not available. OpenCV required for full functionality.")

def _int_to_float(im):
    return im.astype(float) / np.iinfo(im.dtype).max

if _has_opencv:
    def _ycrcb2rgb(img):
        return _int_to_float(cv.cvtColor(img, cv.COLOR_YCrCb2RGB))

else:
    def _ycrcb2rgb(img):
        """ YCrCb to RGB, based on OpenCV documentation definition.

        Note: It seems this implementation is not perfectly equivalent to
        OpenCV's (results not exactly same, why?)
        """

        rgb = np.zeros(img.shape, np.float)

        Y = img[:,:,0].astype(np.float)
        Cr = img[:,:,1].astype(np.float)
        Cb = img[:,:,2].astype(np.float)
        delta = 128.0

        rgb[:,:,0] = Y + 1.403 * (Cr - delta)
        rgb[:,:,1] = Y - 0.714 * (Cr - delta) - 0.344 * (Cb - delta)
        rgb[:,:,2] = Y + 1.773 * (Cb - delta)

        return rgb / 255

def _ycbcr2rgb(img):
    rgb = np.zeros(img.shape, np.float)

    Y = img[:,:,0].astype(np.float)
    Cr = img[:,:,2].astype(np.float)
    Cb = img[:,:,1].astype(np.float)
    delta = 128.0

    rgb[:,:,0] = Y + 1.403 * (Cr - delta)
    rgb[:,:,1] = Y - 0.714 * (Cr - delta) - 0.344 * (Cb - delta)
    rgb[:,:,2] = Y + 1.773 * (Cb - delta)

    return rgb / 255

################################################################################
# Decoding
################################################################################

class FTLDecoder:
    def decode(self, packet):
        raise NotImplementedError()

################################################################################
# OpenCV (optional)
################################################################################

def decode_codec_opencv(packet):
    if packet.block_total != 1 or packet.block_number != 0:
        raise Exception("Unsupported block format (todo)")

    return _int_to_float(cv.imdecode(np.frombuffer(packet.data, dtype=np.uint8),
                                                   cv.IMREAD_UNCHANGED))

def decode_codec_opencv_float(packet):
    if packet.block_total != 1 or packet.block_number != 0:
        raise Exception("Unsupported block format (todo)")

    return cv.imdecode(np.frombuffer(packet.data, dtype=np.uint8),
                                     cv.IMREAD_UNCHANGED).astype(np.float) / 1000.0

################################################################################
# HEVC
################################################################################

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

def _get_NAL_type(data):
    if not isinstance(data, bytes):
        raise ValueError("expected bytes")

    return _NALType((data[4] >> 1) & 0x3f)

def _is_iframe(data):
    return _get_NAL_type(data) == _NALType.VPS

class FTLDecoder_HEVC:
    def __init__(self):
        self._decoder = libde265.Decoder()
        self._seen_iframe = False

    def decode(self, packet):
        if not self._seen_iframe:
            if not _is_iframe(packet.data):
                # can't decode before first I-frame has been received
                warn("received P-frame before I-frame")
                return

        self._decoder.push_data(packet.data)
        self._decoder.push_end_of_frame()

        while self._decoder.get_number_of_input_bytes_pending() > 0:
            self._decoder.decode()

        img = self._decoder.get_next_picture()
        if img is None:
            # if this happens, does get_next_picture() in loop help?
            warn("frame expected, no image received from decoder")

        return img

class FTLDecoder_HEVC_Float(FTLDecoder_HEVC):
    @staticmethod
    def _decode_format_nvpipe(img):
        high = img[:,(img.shape[1]//2):,0].astype(np.uint32) << 8
        low = img[:,:(img.shape[1]//2),0].astype(np.uint32)
        return (high|low).astype(np.float)/1000.0

    @staticmethod
    def _decode_format_mappeddepth(img):
        # hardcoded constants maxdepth and P
        maxdepth = 16
        P = (2.0 * 256.0) / 16384.0

        # use only 8 bits of 10
        img = (img >> 2).astype(np.float) / 255

        L = img[:,:,0]
        Ha = img[:,:,1]
        Hb = img[:,:,2]

        m = np.floor(4.0 * (L/P) - 0.5).astype(np.int) % 4
        L0 = L - ((L-(P / 8.0)) % P) + (P / 4.0) * m.astype(np.float) - (P/8.0)

        s = np.zeros(img.shape[:2], dtype=np.float)
        np.copyto(s, (P/2.0) * Ha, where=m == 0)
        np.copyto(s, (P/2.0) * Hb, where=m == 1)
        np.copyto(s, (P/2.0) * (1.0 - Ha), where=m == 2)
        np.copyto(s, (P/2.0) * (1.0 - Hb), where=m == 3)

        return (L0 + s) * maxdepth

    def decode(self, packet):
        img = super().decode(packet)
        if img is None:
            return None

        if (packet.flags & ftltype.PacketFlags.MappedDepth):
            return self._decode_format_mappeddepth(img)

        else:
            return self._decode_format_nvpipe(img)

class FTLDecoder_HEVC_YCrCb(FTLDecoder_HEVC):
    def decode(self, packet):
        img = super().decode(packet)
        if img is None:
            return None

        return _ycrcb2rgb(img)

class FTLDecoder_HEVC_YCbCr(FTLDecoder_HEVC):
    def decode(self, packet):
        img = super().decode(packet)
        if img is None:
            return None

        return _ycbcr2rgb(img)

################################################################################
# Other (msgpack/calibration/pose)
################################################################################

def decode_codec_calibration(packet):
    calibration = struct.unpack("@ddddIIdddd", packet.data[:(4*8+2*4+4*8)])
    return Calibration(ftltype.Camera._make(calibration), 0, 0)

def decode_codec_msgpack_calibration(packet):
    calib, channel, capabilities = msgpack.unpackb(packet.data)
    return Calibration(ftltype.Camera._make(calib), channel, capabilities)

def decode_codec_msgpack_pose(packet):
    raw = msgpack.unpackb(packet.data)
    # TODO: msgpack returns 128 (4*4*sizeof(double)) floating point values
    return raw

def decode_codec_pose(packet):
    pose = np.asarray(struct.unpack("@16d", packet.data[:(16*8)]),
                      dtype=np.float64)

    return pose.reshape((4, 4), order="F") # Eigen

################################################################################

def create_decoder(codec, channel, version=3):
    """ @brief Create decoder for given channel, codec and ftlf version.
        @param      codec       Codec id
        @param      channel     Channel id
        @param      version     FTL file version
        @returns    callable which takes packet as argument
    """

    if codec == ftltype.codec_t.HEVC:
        if ftltype.is_float_channel(channel):
            return FTLDecoder_HEVC_Float().decode
        else:
            if version < 3:
                return FTLDecoder_HEVC_YCrCb().decode
            else:
                return FTLDecoder_HEVC_YCbCr().decode

    elif codec == ftltype.codec_t.PNG:
        if not _has_opencv:
            raise Exception("OpenCV required for OpenCV (png/jpeg) decoding")

        if ftltype.is_float_channel(channel):
            return decode_codec_opencv_float
        else:
            return decode_codec_opencv

    elif codec == ftltype.codec_t.JPG:
        if not _has_opencv:
            raise Exception("OpenCV required for OpenCV (png/jpeg) decoding")

        return decode_codec_opencv

    elif codec == ftltype.codec_t.MSGPACK:
        if channel == ftltype.Channel.Calibration:
            return decode_codec_msgpack_calibration
        elif channel == ftltype.Channel.Pose:
            return decode_codec_msgpack_pose
        else:
            return lambda packet: msgpack.unpackb(packet.data)

    elif codec == ftltype.codec_t.CALIBRATION:
        return decode_codec_calibration

    elif codec == ftltype.codec_t.POSE:
        return decode_codec_pose

    else:
        raise ValueError("Unknown codec %i" % codec)

################################################################################
# ENCODING
################################################################################

def create_packet(codec, definition, flags, data):
    return ftltype.Packet._make((codec, definition, 1, 0, flags, data))

# TODO exception types?

def encode_codec_opencv_jpg(data, **kwargs):
    params = []
    retval, encoded = cv.imencode(".jpg", data, params)
    if retval:
        return create_packet(ftltype.codec_t.JPG,
                             ftltype.get_definition(data),
                             0,
                             encoded)
    else:
        # todo
        raise Exception("encoding error")

def encode_codec_opencv_png(data, **kwargs):
    params = [cv.IMWRITE_PNG_COMPRESSION, 9]
    retval, encoded = cv.imencode(".png", data, params)
    if retval:
        return create_packet(ftltype.codec_t.PNG,
                             ftltype.get_definition(data),
                             0,
                             encoded)
    else:
        # todo
        raise Exception("encoding error")

def encode_codec_opencv_png_float(data, compression=9):
    data = (data * 1000).astype(np.uint16)
    params = [cv.IMWRITE_PNG_COMPRESSION, compression]
    retval, encoded = cv.imencode(".png", data, params)
    if retval:
        return create_packet(ftltype.codec_t.PNG,
                             ftltype.get_definition(data),
                             0,
                             encoded)
    else:
        # todo
        raise Exception("encoding error")

def create_encoder(codec, channel, **options):
    """ @brief  Create encoder
        @param      codec       codec id
        @param      channel     channel id
        @param      **options   options passed to codec constructor
        @returns    callable which takes unencoded data and optional parameters
    """

    if codec == ftltype.codec_t.JPG:
        if not ftltype.is_float_channel(channel):
            return encode_codec_opencv_jpg
        else:
            raise Exception("JPG not supported for float channels")

    elif codec == ftltype.codec_t.PNG:
        if ftltype.is_float_channel(channel):
            return encode_codec_opencv_png_float
        else:
            return encode_codec_opencv_png

    elif codec == ftltype.codec_t.MSGPACK:
        if channel == ftltype.Channel.Pose:
            raise NotImplementedError("todo")

        elif channel == ftltype.Channel.Calibration:
            raise NotImplementedError("todo")

        else:
            raise Exception("msgpack only available for pose/calibration")

    else:
        raise Exception("unsupported/unknown codec")
