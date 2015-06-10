
import unittest
import sys


class ExampleTest(unittest.TestCase):
    def test_python_version(self):
        self.assertEqual(sys.version_info[0], 3)
        self.assertGreaterEqual(sys.version_info[1], 3)
