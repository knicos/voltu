import unittest
import tempfile
import os

from ftl import FTLStreamWriter
from ftl.types import Channel, Camera, FTLException

import numpy as np

class TestStreamWriter(unittest.TestCase):

    def test_create_and_delete_file(self):
        """ Test constructor and destructor """

        f = tempfile.NamedTemporaryFile(suffix=".ftl")
        stream = FTLStreamWriter(f.name)
        stream.close()
        stream.__del__()

    def test_write_frames_float32_1080p(self):
        """ Write random image, correct types and values """

        f = tempfile.NamedTemporaryFile(suffix=".ftl")
        with FTLStreamWriter(f.name) as stream:
            calib = Camera(700.0, 700.0, 960.0, 540.0, 1920, 1080, 0.0, 10.0, 0.20, 0.0)
            im = np.random.rand(1080, 1920, 3).astype(np.float32)

            stream.write(0, Channel.Calibration, calib)
            stream.write(0, Channel.Colour, im)

    def test_write_calib_wrong_compatible_type(self):
        """ Write calibration with incorrect but compatible types (float/int) """
        f = tempfile.NamedTemporaryFile(suffix=".ftl")

        with FTLStreamWriter(f.name) as stream:
            calib = Camera(700, 700.0, 960, 540.0, 1920.0, 1080, 0, 10.0, 0.2, 0)
            stream.write(0, Channel.Calibration, calib)

    def test_write_calib_wrong_incompatible_type(self):
        """ Write calibration with incorrect and incompatible types """

        f = tempfile.NamedTemporaryFile(suffix=".ftl")

        with FTLStreamWriter(f.name) as stream:
            calib = Camera("foo", "bar", 960, 540.0, 1920.0, 1080, 0, 10.0, 0.2, 0)
            with self.assertRaises(ValueError):
                stream.write(0, Channel.Calibration, calib)

    def test_empty_nextframe(self):
        """ Call nextframe() on empty stream """

        f = tempfile.NamedTemporaryFile(suffix=".ftl")
        with FTLStreamWriter(f.name) as stream:
            with self.assertRaises(FTLException):
                stream.next_frame()

if __name__ == '__main__':
    unittest.main()