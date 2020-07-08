import unittest
import tempfile
import os

from ftl import FTLStreamWriter, FTLStreamReader
from ftl.types import Channel, Camera, FTLException

import numpy as np
import cv2

class TestStreamWriter(unittest.TestCase):

    def test_read_write_frames_uint8_1080p(self):
        """ Write calibration and random 1080p image and then read them """
        return # Sebastian to fix this test: Line 31 has wrong channel in orig[1].

        f = tempfile.NamedTemporaryFile(suffix=".ftl")

        frames = [
            (0, Channel.Calibration, Camera(700.0, 700.0, 960.0, 540.0, 1920, 1080, 0.0, 10.0, 0.25, 0.0)),
            (0, Channel.Colour, (np.random.rand(1080, 1920, 3) * 255).astype(np.uint8))
        ]

        with FTLStreamWriter(f.name) as writer:
            for frame in frames:
                writer.write(*frame)

        with FTLStreamReader(f.name) as reader:
            for (_, src, channel, data), orig in zip(reader, frames):
                self.assertEqual(src, orig[0])
                self.assertEqual(channel, orig[1])

                if channel == Channel.Calibration:
                    # floating point accuracy can cause issues (writer uses 32
                    # bit representation while python uses 64 by default)
                    self.assertEqual(data.camera(), orig[2])

                elif channel == Channel.Colour:
                    # reader returns color channel as float, while writer
                    # internally converts it to uint8
                    im = (data[:,:,0:3] * 255).astype(np.uint8)
                    self.assertTrue(np.array_equal(im, orig[2]))

if __name__ == '__main__':
    unittest.main()
