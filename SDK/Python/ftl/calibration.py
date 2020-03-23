import numpy as np
from . types import Camera

def get_camera_matrix(calib):
    K = np.identity(3, dtype=np.float64)
    K[0,0] = calib.fx
    K[1,1] = calib.fy
    K[0,2] = -calib.cx
    K[1,2] = -calib.cy
    return K

def get_Q(calib):
    """ Disparity to depth matrix. Explained in "Learning OpenCV: Computer
        Vision with the OpenCV Library" (2008) p. 435.
    """
    Q = np.identity(4, dtype=np.float64)
    Q[0,3] = calib.cx
    Q[1,3] = calib.cy
    Q[2,2] = 0.0
    Q[2,3] = calib.fx
    Q[3,2] = -1 / calib.baseline
    Q[3,3] = calib.doff
    return Q

class Calibration:
    @staticmethod
    def from_K(K, size, min_depth=0.0, max_depth=100.0, baseline=0.0, doff=0.0):
        calib = Camera._make([K[0,0], K[1,1], K[0,2], K[1,2], size[1], size[0],
                             min_depth, max_depth, baseline, doff])
        return Calibration(calib, None, None)

    def __init__(self, calib, channel, capabilities):
        self._calib = calib
        self._capabilities = capabilities

    def matrix(self):
        return get_camera_matrix(self._calib)

    def Q(self):
        return get_Q(self._calib)

    def camera(self):
        return self._calib
