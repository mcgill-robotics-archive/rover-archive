#!/usr/bin/env python
import math

__author__ = 'David Lavoie-Boutin'


def angle_mod(n):
    """
    Keep angle in the 0 to 2 pi range

    :param n: angle to be mapped
    :return: same angle converted the the 0 to 2 i range
    """
    return divmod(n, 2 * math.pi)[1]


def max_mag(numbers):
    """
    Function finding the item in the list with maximum magnitude, or absolute value
    :param numbers: list of elements (normally numbers)
    :return: element of the list with largest magnitude
    """
    greatest = 0
    for x in numbers:
        if abs(x) > abs(greatest):
            greatest = x
    return greatest


# function returns the sign of a variable (1,0 or -1)
def sign(n):
    """
    Function to find the sign of an element
    :param n: Element which we want to find the sign of
    :return: The sign of the element in unit form (-1, 0, 1)
    """
    if n == 0:
        return 0.
    else:
        return float(n) / abs(float(n))
