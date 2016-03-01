__author__ = 'David'

from armikp import *
import numpy as np

rot = np.matrix([[1,1,1],[-2,1,0],[-3,10,0]])
print rot
armpikp([100,0,400],rot)