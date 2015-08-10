
import unittest
import os


def everything():
    return unittest.TestLoader().discover(os.path.dirname(__file__),
                                          pattern='test_*.py')
