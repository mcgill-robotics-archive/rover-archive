"""Basic calculation of the center of rotation based on goal position.
See https://www.desmos.com/calculator/7i8tk6johu for an understanding of what this is doing.
"""

import math
import unittest


def calculate_center_of_rotation(goal_coordinates):
    """Produce the point around which we must turn to get to (goal_x, goal_y).
    The assumption here is that we are currently at (0,0) orriented toward
    the positive x axis.
    The center of rotation is the zero crossing of the x axis of the bisection
    of the vector connecting the (0,0) to the (goal_x, goal_y).
    """
    goal_x = float(goal_coordinates[0])
    goal_y = float(goal_coordinates[1])
    if goal_y == 0:
        turn_center = (0, float("inf"))
    else:
        bisection_slope = -1 * goal_x / goal_y
        bisection_x = goal_x / 2
        bisection_y = goal_y / 2

        # The turn center is the zero crossing of the x axis of the bisection.
        turn_center = (0, -1 * bisection_x * bisection_slope + bisection_y)

    return turn_center


class TestCenterOfRotationCalculator(unittest.TestCase):

    def test_infinity_case(self):
        coords = (1, 0)
        res = (0, float("inf"))
        self.assertEqual(calculate_center_of_rotation(coords), res)

    def test_known_cases(self):
        coords = (2, 2)
        res = (0, 2)
        self.assertEqual(calculate_center_of_rotation(coords), res)
        coords = (6, 2)
        res = (0, 10)
        self.assertEqual(calculate_center_of_rotation(coords), res)
        coords = (6, 8)
        res = (0, 6.25)
        self.assertEqual(calculate_center_of_rotation(coords), res)
#test for negative coordinates
        coords = (-4, 5)
        res = (0, 4.1)
        self.assertEqual(calculate_center_of_rotation(coords), res)
        coords = (-4, -5)
        res = (0, -4.1)
        self.assertEqual(calculate_center_of_rotation(coords), res)
if __name__ == "__main__":
	unittest.main()
