import unittest

class LoadLibrary(unittest.TestCase):

    def test_get_instance(self):
        import voltu
        self.assertIsNotNone(voltu.instance())
        # second call to instance() returns None
        #self.assertIsNotNone(voltu.instance())

if __name__ == '__main__':
    unittest.main()
