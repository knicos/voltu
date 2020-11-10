import unittest
import voltu
import numpy as np

class Intrinsics(unittest.TestCase):

    def test_default_ctor(self):
        intr = voltu.Intrinsics()
        self.assertIsNotNone(intr)

    def test_create(self):
        fx = 2.0
        fy = 3.0
        cx = 4.0
        cy = 5.0
        w = 6
        h = 7

        intr = voltu.Intrinsics(
            width = w,
            height = h,
            focal_x = fx,
            focal_y = fy,
            principle_x = -cx,
            principle_y = -cy
        )

        K = np.array([
            [fx , 0.0,  cx],
            [0.0,  fy,  cy],
            [0.0, 0.0, 1.0],
        ])

        self.assertTrue(np.array_equal(intr.matrix(), K))

        w_, h_ = intr.size()
        self.assertEqual(w_, w)
        self.assertEqual(h_, h)
        self.assertTrue(np.array_equal((w, h), intr.size()))
