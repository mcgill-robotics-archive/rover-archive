#!/usr/bin/env python

"""TestWheelOutputData: Collection of tests for the WheelOutputData class"""

import unittest
from WheelOutputData import WheelOutputData

__author__ = "Erica Dep, Mark Zhu"

class TestWheelOutputData(unittest.TestCase):

    #testing init
    def test_init(self):
        x = WheelOutputData()
        self.assertEqual(x.flsa, 0)
        self.assertEqual(x.frsa, 0)
        self.assertEqual(x.blsa, 0)
        self.assertEqual(x.brsa, 0)
        self.assertEqual(x.flv, 0)
        self.assertEqual(x.frv, 0)
        self.assertEqual(x.mlv, 0)
        self.assertEqual(x.mrv, 0)
        self.assertEqual(x.blv, 0)
        self.assertEqual(x.brv, 0)

    #testing set_velocity_zero
    def test_set_velocity_zero(self):
        x = WheelOutputData()
        WheelOutputData.set_velocity_zero(x)
        self.assertEqual(x.flv, 0)
        self.assertEqual(x.frv, 0)
        self.assertEqual(x.mlv, 0)
        self.assertEqual(x.mrv, 0)
        self.assertEqual(x.blv, 0)
        self.assertEqual(x.brv, 0)

    #testing set_angle_zero
    def test_set_angle_zero(self):
        x = WheelOutputData()
        WheelOutputData.set_angle_zero(x)
        self.assertEqual(x.flsa, 0)
        self.assertEqual(x.frsa, 0)
        self.assertEqual(x.blsa, 0)
        self.assertEqual(x.brsa, 0)

    #testing create_message FIX
    def test_create_message(self):
	x = WheelOutputData()
        x.flsa = 1
        x.frsa = 2
        x.blsa = 3
        x.brsa = 4
        x.flv = 5
        x.frv = 6
        x.mlv = 7
        x.mrv = 8
        x.blv = 9
        x.brv = 10
	command = WheelOutputData.create_message(x)
        self.assertEqual(command.flsa, 1)
        self.assertEqual(command.frsa, 2)
        self.assertEqual(command.blsa, 3)
        self.assertEqual(command.brsa, 4)
        self.assertEqual(command.flv, 5*11)
        self.assertEqual(command.frv, 6*11)
        self.assertEqual(command.mlv, 7*11)
        self.assertEqual(command.mrv, 8*11)
        self.assertEqual(command.blv, 9*11)
        self.assertEqual(command.brv, 10*11)

    #testing create_message w/ 0 values FIX
    def test_create_message_zeroes(self):
        x = WheelOutputData()
	command = WheelOutputData.create_message(x)
        self.assertEqual(command.flsa, 0)
        self.assertEqual(command.frsa, 0)
        self.assertEqual(command.blsa, 0)
        self.assertEqual(command.brsa, 0)
        self.assertEqual(command.flv, 0)
        self.assertEqual(command.frv, 0)
        self.assertEqual(command.mlv, 0)
        self.assertEqual(command.mrv, 0)
        self.assertEqual(command.blv, 0)
        self.assertEqual(command.brv, 0)

    #testing set_data
    def test_set_data(self):
        x = WheelOutputData()
        WheelOutputData.set_data(x, [1,2,3,4,5,6,7,8,9,0])
        self.assertEqual(x.flsa, 1)
        self.assertEqual(x.frsa, 2)
        self.assertEqual(x.blsa, 3)
        self.assertEqual(x.brsa, 4)
        self.assertEqual(x.flv, 5)
        self.assertEqual(x.frv, 6)
        self.assertEqual(x.mlv, 7)
        self.assertEqual(x.mrv, 8)
        self.assertEqual(x.blv, 9)
        self.assertEqual(x.brv, 0)

    #testing set_data with an array of lenght != 10
 #   def test_set_data_wrong_length(self):
  #      x = WheelOutputData()
   #     with self.assertRaises(KeyError) as error:
	#	WheelOutputData.set_data(x, [1,2,3])
#	self.AssertTrue('Bad data set received' in error.exception)

if __name__ == '__main__':
    unittest.main()
