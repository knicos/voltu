'''
Python wrapper for libde265. Only decoding is (partly) implemented.

Requirements:
 * libde265 library (libde265.so.0)
 * numpy
 * skimage (rescaling)

'''

import ctypes

import numpy as np
from skimage.transform import rescale

# error codes copied from header (de265.h)

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

libde265 = ctypes.cdll.LoadLibrary("libde265.so.0")

libde265.de265_get_error_text.argtypes = [ctypes.c_void_p]
libde265.de265_get_error_text.restype = ctypes.c_char_p
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

libde265.de265_flush_data.argtypes = [ctypes.c_void_p]
libde265.de265_flush_data.restype = ctypes.c_int

libde265.de265_decode.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_int)]
libde265.de265_decode.restype = ctypes.c_int

libde265.de265_get_next_picture.argtypes = [ctypes.c_void_p]
libde265.de265_get_next_picture.restype = ctypes.c_void_p

libde265.de265_get_image_width.argtypes = [ctypes.c_void_p, ctypes.c_int]
libde265.de265_get_image_width.restype = ctypes.c_int

libde265.de265_get_image_height.argtypes = [ctypes.c_void_p, ctypes.c_int]
libde265.de265_get_image_height.restype = ctypes.c_int

libde265.de265_get_bits_per_pixel.argtypes = [ctypes.c_void_p, ctypes.c_int]
libde265.de265_get_bits_per_pixel.restype = ctypes.c_int

libde265.de265_get_image_plane.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_int)]
libde265.de265_get_image_plane.restype = ctypes.POINTER(ctypes.c_char)

class Decoder:
    def __init__(self, size, threads=1):
        self.size_ = size
        self.more_ = ctypes.c_int()
        self.out_stride_ = ctypes.c_int()
        self.ctx_ = libde265.de265_new_decoder()
        err = libde265.de265_start_worker_threads(self.ctx_, threads)
        if err:
            raise Exception(self.get_error_str(err))
        
    def __del__(self):
        libde265.de265_free_decoder(self.ctx_)
        
    def get_error_str(self, code):
        return libde265.de265_get_error_text(code).decode("ascii")
    
    def push_data(self, data):
        if not isinstance(data, bytes):
            raise ValueError("expected bytes")
        
        err = libde265.de265_push_data(self.ctx_, data, len(data), None, None)
        
        if err:
            raise Exception(self.get_error_str(err))
            
    def push_end_of_frame(self):
        err = libde265.de265_push_end_of_frame(self.ctx_)
        
        if err:
            raise Exception(self.get_error_str(err))
            
    def push_NAL(self, data):
        if not isinstance(data, bytes):
            raise ValueError("expected bytes")
        
        err = libde265.de265_push_NAL(self.ctx_, data, len(data), None, None)
        
        if err:
            raise Exception(self.get_error_str(err))
            
    def decode(self):
        err = libde265.de265_decode(self.ctx_, self.more_)
        
        if err and err != DE265_ERROR_WAITING_FOR_INPUT_DATA:
            raise Exception(self.get_error_str(err))
            
        return self.more_.value != 0
    
    def flush_data(self):
        libde265.flush_data(self.ctx_)
        
    def get_next_picture(self):
        '''
        Returns next decoded frame. Image in YCrCb format. If no frame available
        returns None.

        TODO: Skimage's ycrcb2rgb() does not look correct. Is Cr/Cb channel
              created correctly?
        '''
        img = libde265.de265_get_next_picture(self.ctx_)
        
        if not img:
            return None
        
        res = np.zeros((self.size_[0], self.size_[1], 3), dtype=np.uint8)
        
        for c in range(0, 3):
            size = (libde265.de265_get_image_height(img, c), libde265.de265_get_image_width(img, c))
            bpp = libde265.de265_get_bits_per_pixel(img, c)
            img_ptr = libde265.de265_get_image_plane(img, c, self.out_stride_)
            scale_y = self.size_[0] / size[0]
            scale_x = self.size_[1] / size[1]
            
            if bpp != 8:
                raise NotImplementedError("unsupported bits per pixel %i" % bpp)
            
            if scale_x != scale_y:
                raise NotImplementedError("unsupported subsampling")
            
            channel = np.asarray(bytearray(img_ptr[:size[0] * size[1]]), dtype=np.uint8)
            channel.shape = size
            res[:,:,c] = (rescale(channel, scale_x) * 255).astype(np.uint8)

        return res