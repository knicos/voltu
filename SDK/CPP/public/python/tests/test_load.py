import unittest
import os
import voltu

class LoadLibrary(unittest.TestCase):

    def test_get_instance(self):
        self.assertIsNotNone(voltu.instance())
        # second call to instance() returns None; should
        # return same instance instead?
        # self.assertIsNotNone(voltu.instance())

    def test_version(self):
        major, minor, patch = voltu.version

if __name__ == '__main__':
    unittest.main()
