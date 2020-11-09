import unittest
import os

class LoadLibrary(unittest.TestCase):

    def test_import(self):
        import voltu

    def test_import_twice(self):
        # verify that System instance is created just once, even if module
        # imported multiple times
        import voltu
        import voltu
        self.assertIsNotNone(voltu.System)

    def test_version(self):
        import voltu
        major, minor, patch = voltu.version

if __name__ == '__main__':
    unittest.main()
