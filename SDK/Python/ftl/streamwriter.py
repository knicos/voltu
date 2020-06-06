from . types import FTLException, Channel, is_float_channel, Camera, Pipeline
import numpy as np
from enum import IntEnum

import ctypes
import sys
import os.path

################################################################################
# Try to locate shared library

_paths = [
    ".",
    "/usr/lib",
    "/usr/local/lib",
]

_libpath = None
if "FTL_LIB" in os.environ and os.path.exists(os.environ["FTL_LIB"]):
    _libpath = os.environ["FTL_LIB"]

else:
    for p in _paths:
        p = os.path.join(p, "libftl-dev.so")
        if os.path.exists(p):
            _libpath = p
            break

if _libpath is None:
    raise FileNotFoundError("libftl-dev.so not found")

################################################################################

class _imageformat_t(IntEnum):
	FLOAT     = 0
	BGRA      = 1
	RGBA      = 2
	RGB       = 3
	BGR       = 4
	RGB_FLOAT = 5

_c_api = ctypes.CDLL(_libpath)

_c_api.ftlCreateWriteStream.restype = ctypes.c_void_p
_c_api.ftlCreateWriteStream.argtypes = [ctypes.c_char_p]

_c_api.ftlIntrinsicsWriteLeft.restype = ctypes.c_int
_c_api.ftlIntrinsicsWriteLeft.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float]

_c_api.ftlIntrinsicsWriteRight.restype = ctypes.c_int
_c_api.ftlIntrinsicsWriteRight.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float]

_c_api.ftlImageWrite.restype = ctypes.c_int
_c_api.ftlImageWrite.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_void_p]

_c_api.ftlPoseWrite.restype = ctypes.c_int
_c_api.ftlPoseWrite.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p]

_c_api.ftlRemoveOcclusion.restype = ctypes.c_int
_c_api.ftlRemoveOcclusion.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_void_p]

_c_api.ftlMaskOcclusion.restype = ctypes.c_int
_c_api.ftlMaskOcclusion.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int, ctypes.c_int, ctypes.c_void_p]

_c_api.ftlEnablePipeline.restype = ctypes.c_int
_c_api.ftlEnablePipeline.argtypes = [ctypes.c_void_p, ctypes.c_int]

_c_api.ftlDisablePipeline.restype = ctypes.c_int
_c_api.ftlDisablePipeline.argtypes = [ctypes.c_void_p, ctypes.c_int]

_c_api.ftlSelect.restype = ctypes.c_int
_c_api.ftlSelect.argtypes = [ctypes.c_void_p, ctypes.c_int]

_c_api.ftlNextFrame.restype = ctypes.c_int
_c_api.ftlNextFrame.argtypes = [ctypes.c_void_p]

_c_api.ftlDestroyStream.restype = ctypes.c_int
_c_api.ftlDestroyStream.argtypes = [ctypes.c_void_p]

def _ftl_check(retval):
    if retval != 0:
        raise FTLException(retval ,"FTL api returned %i" % retval)

class FTLStreamWriter:
    def __init__(self, fname):
        self._sources = {}

        if isinstance(fname, str):
            fname_ = bytes(fname, sys.getdefaultencoding())

        elif isinstance(fname, bytes):
            fname_ = fname

        else:
            raise TypeError()

        self._instance = _c_api.ftlCreateWriteStream(fname_)

        if self._instance is None:
            raise Exception("Error: ftlCreateWriteStream")

    def close(self):
        if self._instance is not None:
            _ftl_check(_c_api.ftlDestroyStream(self._instance))
            self._instance = None

    def __del__(self):
        if self._instance is not None:
            _c_api.ftlDestroyStream(self._instance)

    def __enter__(self):
        return self

    def __exit__(self, type, value, traceback):
        self.close()

    def _check_image(self, source, channel, data):
        """ Check image is has correct number of channels and correct size
             and convert to compatible datatype if necessary. Raises
             TypeError/ValueError on failure.
        """

        if not type(data) is np.ndarray:
            raise TypeError("Data must be in numpy array")

        if source not in self._sources:
            raise ValueError("Source must be configured before adding images")

        if len(data.shape) not in [2, 3]:
            raise ValueError("Unsupported array shape %s" % str(data.shape))

        height, width = self._sources[source].height, self._sources[source].width
        if data.shape[0] != height or data.shape[1] != width:
            # TODO: this can happen (depth and color), requires support in C api
            raise ValueError("Image size different than previously configured")

        if is_float_channel(channel):
            data = data.astype(np.float32)
            ftl_dtype = _imageformat_t.FLOAT

        else:
            if len(data.shape) == 2:
                raise ValueError("Expected multi-channel image for channel %s" % str(channel))

            nchans = data.shape[2]
            if data.dtype in [np.float32, np.float64]:
                data = data.astype(np.float32)
                ftl_dtype = _imageformat_t.RGB_FLOAT
                if nchans != 3:
                    raise ValueError("Unsupported number of channels: %i" % nchans)

            elif data.dtype in [np.int8, np.uint8]:
                if nchans == 3:
                    ftl_dtype = _imageformat_t.BGR
                elif nchans == 4:
                    ftl_dtype = _imageformat_t.BGRA
                else:
                    raise ValueError("Unsupported number of channels: %i" % nchans)

            else:
                raise ValueError ("Unsupported numpy data type")

        data = np.ascontiguousarray(data)
        return data, ftl_dtype

    def _write_image(self, source, channel, data):
        """ Wrapper for ftlImageWrite """

        data, ftl_dtype = self._check_image(source, channel, data)
        _ftl_check(_c_api.ftlImageWrite(self._instance, ctypes.c_int(source),
                                        channel, ftl_dtype, 0,
                                        data.ctypes.data_as(ctypes.c_void_p)))

    def _write_calibration(self, source, channel, camera):
        """ Wrapper for ftlIntrinsicsWriteLeft and ftlIntrinsicsWriteRight """

        # TODO: type checks for camera

        func = _c_api.ftlIntrinsicsWriteLeft if channel == Channel.Calibration else _c_api.ftlIntrinsicsWriteRight

        _ftl_check(func(self._instance, source,
                        int(camera.width), int(camera.height),
                        float(camera.fx), float(camera.cx),
                        float(camera.cy), float(camera.baseline),
                        float(camera.min_depth), float(camera.max_depth)))

        self._sources[source] = camera

    def _write_pose(self, source, channel, pose):
        """ Wrapper for ftlftlPoseWrite """

        if type(pose) not in [np.ndarray, np.matrix]:
            raise TypeError("Data must be in numpy array or matrix")

        if len(pose.shape) != 2 or pose.shape[0] != 4 or pose.shape[1] != 4:
            raise ValueError("Pose must be a 4x4 matrix")

        pose = np.ascontiguousarray(pose.astype(np.float32).T)
        _ftl_check(_c_api.ftlPoseWrite(self._instance, source, pose.ctypes.data_as(ctypes.c_void_p)))

    def write(self, source, channel, data):
        """ Write data to stream """
        source = int(source)
        channel = Channel(channel)

        if channel in [Channel.Calibration, Channel.Calibration2]:
            self._write_calibration(source, channel, data)
        elif channel == Channel.Pose:
            self._write_pose(source, channel, data)
        else:
            self._write_image(source, channel, data)

    def enable_pipeline(self, t):
        if t not in Pipeline:
            raise ValueError("Unknown pipeline")

        _ftl_check(_c_api.ftlEnablePipeline(self._instance, t))

    def disable_pipeline(self, t):
        if t not in Pipeline:
            raise ValueError("Unknown pipeline")

        _ftl_check(_c_api.ftlDisablePipeline(self._instance, t))

    def mask_occlusion(self, source, channel, data):
        data, ftl_dtype = self._check_image(source, channel, data)

        if not is_float_channel(channel):
            raise ValueError("Bad channel")

        if len(data.shape) != 2:
            raise ValueError("Wrong number of channels")

        _ftl_check(_c_api.ftlMaskOcclusion(self._instance, source, channel, 0,
                                           data.ctypes.data_as(ctypes.c_void_p)))

    def next_frame(self):
        _ftl_check(_c_api.ftlNextFrame(self._instance))
