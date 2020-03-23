"""
Python wrapper for libde265. Only decoding is implemented.

Requirements:
 * libde265 library (libde265.so.0)
 * numpy
 * opencv (recommended) or skimage

"""

try:
    import cv2 as cv
    def _resize(img, size):
        dst = np.zeros(size, dtype=img.dtype)
        cv.resize(img, tuple(reversed(size)), dst, interpolation=cv.INTER_LINEAR)
        return dst

except ImportError:
    # seems to be much slower than OpenCV resize()

    from skimage.transform import resize as resize_skimage
    def _resize(img, size):
        # skimage resize() return dtype float64, convert back to original type
        # order: 0 nn, 1 bilinear, 3 bicubic
        return (resize_skimage(img, size, order=2, mode="constant", cval=0) *  np.iinfo(img.dtype).max).astype(img.dtype)

from warnings import warn

import ctypes
from enum import IntEnum

import numpy as np

import os


# default number of worker threads for decoder: half of os.cpu_count()
#
#_threads = os.cpu_count() // 2
#if _threads is None:
#    _threads = 1

_threads = 1

################################################################################
# interface and definitions from libde256 api
################################################################################

# error codes copied from header (de265.h)

class _libde265error(IntEnum):
    DE265_OK = 0
    DE265_ERROR_NO_SUCH_FILE=1
    DE265_ERROR_COEFFICIENT_OUT_OF_IMAGE_BOUNDS=4
    DE265_ERROR_CHECKSUM_MISMATCH=5
    DE265_ERROR_CTB_OUTSIDE_IMAGE_AREA=6
    DE265_ERROR_OUT_OF_MEMORY=7
    DE265_ERROR_CODED_PARAMETER_OUT_OF_RANGE=8
    DE265_ERROR_IMAGE_BUFFER_FULL=9
    DE265_ERROR_CANNOT_START_THREADPOOL=10
    DE265_ERROR_LIBRARY_INITIALIZATION_FAILED=11
    DE265_ERROR_LIBRARY_NOT_INITIALIZED=12
    DE265_ERROR_WAITING_FOR_INPUT_DATA=13
    DE265_ERROR_CANNOT_PROCESS_SEI=14
    DE265_ERROR_PARAMETER_PARSING=15
    DE265_ERROR_NO_INITIAL_SLICE_HEADER=16
    DE265_ERROR_PREMATURE_END_OF_SLICE=17
    DE265_ERROR_UNSPECIFIED_DECODING_ERROR=18
    DE265_ERROR_NOT_IMPLEMENTED_YET = 502
    DE265_WARNING_NO_WPP_CANNOT_USE_MULTITHREADING = 1000
    DE265_WARNING_WARNING_BUFFER_FULL=1001
    DE265_WARNING_PREMATURE_END_OF_SLICE_SEGMENT=1002
    DE265_WARNING_INCORRECT_ENTRY_POINT_OFFSET=1003
    DE265_WARNING_CTB_OUTSIDE_IMAGE_AREA=1004
    DE265_WARNING_SPS_HEADER_INVALID=1005
    DE265_WARNING_PPS_HEADER_INVALID=1006
    DE265_WARNING_SLICEHEADER_INVALID=1007
    DE265_WARNING_INCORRECT_MOTION_VECTOR_SCALING=1008
    DE265_WARNING_NONEXISTING_PPS_REFERENCED=1009
    DE265_WARNING_NONEXISTING_SPS_REFERENCED=1010
    DE265_WARNING_BOTH_PREDFLAGS_ZERO=1011
    DE265_WARNING_NONEXISTING_REFERENCE_PICTURE_ACCESSED=1012
    DE265_WARNING_NUMMVP_NOT_EQUAL_TO_NUMMVQ=1013
    DE265_WARNING_NUMBER_OF_SHORT_TERM_REF_PIC_SETS_OUT_OF_RANGE=1014
    DE265_WARNING_SHORT_TERM_REF_PIC_SET_OUT_OF_RANGE=1015
    DE265_WARNING_FAULTY_REFERENCE_PICTURE_LIST=1016
    DE265_WARNING_EOSS_BIT_NOT_SET=1017
    DE265_WARNING_MAX_NUM_REF_PICS_EXCEEDED=1018
    DE265_WARNING_INVALID_CHROMA_FORMAT=1019
    DE265_WARNING_SLICE_SEGMENT_ADDRESS_INVALID=1020
    DE265_WARNING_DEPENDENT_SLICE_WITH_ADDRESS_ZERO=1021
    DE265_WARNING_NUMBER_OF_THREADS_LIMITED_TO_MAXIMUM=1022
    DE265_NON_EXISTING_LT_REFERENCE_CANDIDATE_IN_SLICE_HEADER=1023
    DE265_WARNING_CANNOT_APPLY_SAO_OUT_OF_MEMORY=1024
    DE265_WARNING_SPS_MISSING_CANNOT_DECODE_SEI=1025
    DE265_WARNING_COLLOCATED_MOTION_VECTOR_OUTSIDE_IMAGE_AREA=1026

class de265_chroma(IntEnum):
    de265_chroma_mono = 0
    de265_chroma_420 = 1
    de265_chroma_422 = 2
    de265_chroma_444 = 3

libde265 = ctypes.cdll.LoadLibrary("libde265.so.0")

libde265.de265_get_error_text.argtypes = [ctypes.c_void_p]
libde265.de265_get_error_text.restype = ctypes.c_char_p

libde265.de265_get_warning.argtypes = [ctypes.c_void_p]
libde265.de265_get_warning.restype = ctypes.c_int

libde265.de265_get_version_number_major.restype = ctypes.c_uint32
libde265.de265_get_version_number_minor.restype = ctypes.c_uint32

libde265.de265_new_decoder.restype = ctypes.c_void_p

libde265.de265_free_decoder.argtypes = [ctypes.c_void_p]
libde265.de265_free_decoder.restype = ctypes.c_int

libde265.de265_start_worker_threads.argtypes = [ctypes.c_void_p, ctypes.c_int]
libde265.de265_start_worker_threads.restype = ctypes.c_int

libde265.de265_push_data.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p, ctypes.c_void_p]
libde265.de265_push_data.restype = ctypes.c_int

libde265.de265_push_NAL.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int, ctypes.c_void_p, ctypes.c_void_p]
libde265.de265_push_data.restype = ctypes.c_int

libde265.de265_push_end_of_frame.argtypes = [ctypes.c_void_p]
libde265.de265_push_end_of_frame.restype = None

libde265.de265_flush_data.argtypes = [ctypes.c_void_p]
libde265.de265_flush_data.restype = ctypes.c_int

libde265.de265_decode.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_int)]
libde265.de265_decode.restype = ctypes.c_int

libde265.de265_get_next_picture.argtypes = [ctypes.c_void_p]
libde265.de265_get_next_picture.restype = ctypes.c_void_p

libde265.de265_peek_next_picture.argtypes = [ctypes.c_void_p]
libde265.de265_peek_next_picture.restype = ctypes.c_void_p

libde265.de265_release_next_picture.argtypes = [ctypes.c_void_p]
libde265.de265_release_next_picture.restype = None

libde265.de265_get_chroma_format.argtypes = [ctypes.c_void_p]
libde265.de265_get_chroma_format.restype = ctypes.c_int

libde265.de265_get_image_width.argtypes = [ctypes.c_void_p, ctypes.c_int]
libde265.de265_get_image_width.restype = ctypes.c_int

libde265.de265_get_image_height.argtypes = [ctypes.c_void_p, ctypes.c_int]
libde265.de265_get_image_height.restype = ctypes.c_int

libde265.de265_get_bits_per_pixel.argtypes = [ctypes.c_void_p, ctypes.c_int]
libde265.de265_get_bits_per_pixel.restype = ctypes.c_int

libde265.de265_get_image_plane.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_int)]
libde265.de265_get_image_plane.restype = ctypes.POINTER(ctypes.c_char)

libde265.de265_get_number_of_input_bytes_pending.argtypes = [ctypes.c_void_p]
libde265.de265_get_number_of_input_bytes_pending.restype = ctypes.c_int

################################################################################

class libde265Error(Exception):
    def __init__(self, code):
        super(libde265Error, self).__init__(
            libde265.de265_get_error_text(code).decode("ascii"))

class WaitingForInput(libde265Error):
    pass

class Decoder:
    """ Python interface to libde256 decoder API.
    """
    def __init__(self, threads=_threads):
        self._more = ctypes.c_int()
        self._out_stride = ctypes.c_int()
        self._ctx = libde265.de265_new_decoder()
        self._disable_warnings = False

        err = libde265.de265_start_worker_threads(self._ctx, threads)

        if err:
            raise libde265Error(err)

    def __del__(self):
        if self._ctx:
            libde265.de265_free_decoder(self._ctx)

    def _copy_image(self, de265_image):
        size = (libde265.de265_get_image_height(de265_image, 0),
                libde265.de265_get_image_width(de265_image, 0))

        res = np.zeros((*size, 3), dtype=np.uint16)

        # libde265: always 420 (???)
        chroma_format = libde265.de265_get_chroma_format(de265_image)
        if chroma_format != de265_chroma.de265_chroma_420:
            raise NotImplementedError("Unsupported chroma format %s" % str(chroma_format))

        for c in range(0, 3):
            size_channel = (libde265.de265_get_image_height(de265_image, c),
                            libde265.de265_get_image_width(de265_image, c))

            if size_channel[0] > size[0] or size_channel[1] > size[1]:
                raise Exception("Channel larger than first channel")

            bpp = libde265.de265_get_bits_per_pixel(de265_image, c)
            if bpp == 8:
                dtype = np.uint8
            else:
                dtype = np.uint16

            img_ptr = libde265.de265_get_image_plane(de265_image, c, self._out_stride)

            ch = np.frombuffer(img_ptr[:size_channel[0] * self._out_stride.value], dtype=dtype)
            ch.shape = size_channel

            res[:,:,c] = _resize(ch, size)

        return res

    def _warning(self):
        if self._disable_warnings:
            return

        code = libde265.de265_get_warning(self._ctx)

        if code != _libde265error.DE265_OK:
            msg = libde265.de265_get_error_text(code).decode("ascii")
            warn(msg)

    def decode(self):
        err = libde265.de265_decode(self._ctx, self._more)

        if err:
            if err == _libde265error.DE265_ERROR_WAITING_FOR_INPUT_DATA:
                raise WaitingForInput(err)

            raise libde265Error(err)

        self._warning()

        return self._more.value != 0

    def flush_data(self):
        err = libde265.de265_flush_data(self._ctx)

        if err:
            raise libde265Error(err)

    def push_data(self, data):
        """ Push data to decoder

            @param  data    input bytes
        """
        if not isinstance(data, bytes):
            raise ValueError("expected bytes")

        err = libde265.de265_push_data(self._ctx, data, len(data), None, None)

        if err:
            raise libde265Error(err)

    def push_end_of_frame(self):
        err = libde265.de265_push_end_of_frame(self._ctx)

        if err:
            raise libde265Error(err)

    def push_NAL(self, data):
        if not isinstance(data, bytes):
            raise ValueError("expected bytes")

        err = libde265.de265_push_NAL(self._ctx, data, len(data), None, None)

        if err:
            raise libde265Error(err)

    def get_next_picture(self):
        '''
        Get decoded frame.

        @returns image in YCbCr format or None if no frame available
        '''

        de265_image = libde265.de265_get_next_picture(self._ctx)

        if not de265_image:
            return None

        res = self._copy_image(de265_image)

        libde265.de265_release_next_picture(self._ctx)

        return res

    def get_number_of_input_bytes_pending(self):
        return libde265.de265_get_number_of_input_bytes_pending(self._ctx)

    def peek_next_picture(self):
        de265_image = libde265.de265_peek_next_picture(self._ctx)

        if not de265_image:
            return None

        res = self._copy_image(de265_image)

        libde265.de265_release_next_picture(self._ctx)

        return res
