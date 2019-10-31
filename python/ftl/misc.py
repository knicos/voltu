
def disparity_to_depth(disparity, camera, max_depth=10.0, invalid_value=0.0):
    ''' Calculate depth map from disparity map. Depth values smaller than 0.0 
	    and larger than max_depth are set to invalid_value.
    '''
    depth = (camera.fx * camera.baseline) / (disparity - camera.doff)
    depth[depth < 0] = invalid_value
    depth[depth > max_depth] = invalid_value
    return depth

from enum import IntEnum

# components/codecs/include/ftl/codecs/hevc.hpp
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

def is_iframe(data):
    return get_NAL_type(data) == NALType.VPS
