#!/usr/bin/env python

"""TestUtilities: Collection of tests for the Utilities class"""

import unittest
import math
from Utilities import Utilities

__author__ = "Erica Dep"

class TestUtilities(unittest.TestCase):

    #testing angle_mod
    def test_angle_mod(self):
        x = Utilities.angle_mod(3 * math.pi)
        self.assertEqual(x, math.pi)

    #testing max_mag
    def test_max_mag(self):
        list = [1,2,3,4,5]
        y = Utilities.max_mag(list)
        self.assertEqual(y, 5)

    #testing sign
    def test_sign(self):
        x = -35
        y = 0
        z = 72
        self.assertEqual(Utilities.sign(x), -1)
        self.assertEqual(Utilities.sign(y), 0)
        self.assertEqual(Utilities.sign(z), 1)

if __name__ == '__main__':
    unittest.main()

