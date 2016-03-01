#!/usr/bin/env python

import math
import numpy as np

transpose = np.transpose
deg2rad = math.radians

Rot = {0, 0, -1, 0, 0, 1, 0, 0, 1, 0, 0, 0}

maxrot = 0.5
minrot = -0.5
maxRotLimit = {0, 0, 0, 0, 0, 0}
minRotLimit = {0, 0, 0, 0, 0, 0}
Vj = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
dAngle = {0, 0, 0, 0, 0, 0}
lastJ = {}
wristPos = {}
wristStep = 1
lastWristPos = {}
angletol = 0.001
accelLimit = 2
wristcount = 0

jointRanges = [[math.radians(0), math.radians(360)],
               [math.radians(-10), math.radians(120)],
               [math.radians(-65), math.radians(120)],
               [math.radians(0), math.radians(360)],
               [math.radians(70), math.radians(290)],
               [math.radians(-180), math.radians(180)]]


def split(str, sep):
    if sep is None:
        sep = ','

    fields = []

    # match regex
    # add all matches to the fields array
    # return the array
    pass


def armpikp(wrist, rotation):
    a = [0, 457, 0, 0, 0, 0]
    b = [76, 0, 0, 400, 0, 50]
    alpha = [90, 0, 90, 90, 90, 0]
    lam = [math.cos(deg2rad(alpha[0])), math.cos(deg2rad(alpha[1])), math.cos(deg2rad(alpha[2])),
           math.cos(deg2rad(alpha[3])), math.cos(deg2rad(alpha[4])), math.cos(deg2rad(alpha[5]))]
    mu = [math.sin(deg2rad(alpha[0])), math.sin(deg2rad(alpha[1])), math.sin(deg2rad(alpha[2])),
          math.sin(deg2rad(alpha[3])), math.sin(deg2rad(alpha[4])), math.sin(deg2rad(alpha[5]))]

    eps = 0.0000001
    root_tol = 0.01
    theta = []
    eta = [0, 0, 0]
    w = []

    for i in xrange(0, 6):
        theta.append([])
        for j in xrange(0, 8):
            theta[i].append(0.0)

    xc = wrist[0]
    yc = wrist[1]
    zc = wrist[2]

    Q = transpose(rotation)
    e6 = np.multiply(Q,[0,0,1])  # todo fix matrix dimensions

    theta[0][0] = 2 * math.atan((xc + math.sqrt(yc * yc + xc * xc + eps)) / (-yc + eps))
    theta[0][1] = 2 * math.atan((xc - math.sqrt(yc * yc + xc * xc + eps)) / (-yc + eps))
    theta[0][2] = theta[0][0]
    theta[0][3] = theta[0][1]

    theta[2][0] = 2 * math.atan((-2 * a[1] * b[3] * mu[2] + math.sqrt(
        eps + 4 * math.pow(a[1], 2) * math.pow(b[3], 2) * math.pow(mu[2], 2) - math.pow(math.pow(a[1], 2) + math.pow(b[3], 2) - math.pow(xc, 2) - math.pow(yc, 2) - math.pow(zc - b[0], 2), 2))) / ( math.pow(a[1], 2) + math.pow(b[3], 2) - math.pow(xc, 2) - math.pow(yc, 2) - math.pow(zc - b[0], 2) + eps))

    theta[2][1] = theta[2][0]
    theta[2][2] = 2 * math.atan((-2 * a[1] * b[3] - math.sqrt(eps + 4 * math.pow(a[1], 2) * math.pow(b[3], 2) - math.pow(math.pow(a[1], 2) + math.pow(b[3], 2) - math.pow(xc, 2) - math.pow(yc, 2) - math.pow(zc - b[0], 2) , 2))) / (math.pow(a[1], 2) + math.pow(b[3], 2) - math.pow(xc, 2) - math.pow(yc, 2) - math.pow(zc - b[0], 2) + eps))
    theta[2][3] = theta[2][2]

    for i in xrange(0, 4):
        A11 = a[1] + b[3] * math.sin(theta[3][i])
        A12 = b[3] * lam[1] * mu[2] * math.cos(theta[2][i]) + b[2] * mu[1] * lam[2]
        delta2 = A11 * A11 + A12 * A12

        cosroot1 = math.acos(max(min(1 / delta2 * (
            A11 * (xc * math.cos(theta[0][i]) + yc * math.sin(theta[0][i])) - A12 * ((zc - b[0]) * mu[0])), 1), -1))
        cosroot2 = -cosroot1 + 2 * math.pi

        temp_root = (
            1 / delta2 * (
                A12 * (xc * math.cos(theta[0][i]) + yc * math.sin(theta[0][i])) + A11 * ((zc - b[0]) * mu[0])))
        sinroot1 = math.asin(temp_root)

        if temp_root >= 0:
            sinroot2 = math.pi - sinroot1
            sinroot1 = math.asin(temp_root)
        else:
            sinroot1 += 2 * math.pi
            sinroot2 = 3 * math.pi - sinroot1

        if abs(cosroot1 - sinroot1) < root_tol or abs(cosroot1 - sinroot2) < root_tol:
            theta[1][i] = cosroot1
        else:
            theta[1][i] = cosroot2

    for j in xrange(0, 4):
        eta[0] = (math.cos(theta[0][j]) * math.cos(theta[1][j]) * math.cos(theta[2][j]) - math.cos(
            theta[0][j]) * math.sin(theta[1][j]) * math.sin(theta[2][j])) * e6[0] + (math.sin(theta[0][j]) * math.cos(
            theta[1][j]) * math.cos(theta[2][j]) - math.sin(theta[0][j]) * math.sin(theta[1][j]) * math.sin(
            theta[2][j])) * e6[1] + (math.cos(theta[1][j]) * math.sin(theta[2][j]) + math.sin(theta[1][j]) * math.cos(
            theta[2][j])) * e6[2]

        eta[1] = math.sin(theta[0][j]) * e6[0] - math.cos(theta[0][j]) * e6[1]

        eta[2] = (math.cos(theta[0][j]) * math.cos(theta[1][j]) * math.sin(theta[2][j]) + math.cos(
            theta[0][j]) * math.sin(theta[1][j]) * math.cos(theta[2][j])) * e6[0] + (math.sin(theta[0][j]) * math.cos(
            theta[1][j]) * math.sin(theta[2][j]) + math.sin(theta[0][j]) * math.sin(theta[1][j]) * math.cos(
            theta[2][j])) * e6[1] + (math.sin(theta[1][j]) * math.sin(theta[2][j]) - math.cos(theta[1][j]) * math.cos(
            theta[2][j])) * e6[2]

        theta[3][j] = 2 * math.atan((eta[0] * mu[3] + math.sqrt((math.pow(eta[0], 2) + math.pow(eta[1], 2)) * math.pow(mu[3], 2) - math.pow(lam[4] - eta[2] * lam[3], 2))) /(lam[4] - eta[2] * lam[3] - eta[1] * mu[3]))
        # todo: catch imaginary part of the sqrt

        theta[3][j + 4] = 2 * math.atan(
            (eta[0] * mu[3] - math.sqrt((math.pow(eta[0], 2) + math.pow(eta[1], 2)) * math.pow(mu[3], 2) - math.pow(lam[4] - eta[2] * lam[3], 2))) /
            (lam[4] - eta[2] * lam[3] - eta[1] * mu[3]))

        Q3Q2Q1 = [((math.cos(theta[1][j]) * math.cos(theta[2][j]) - math.sin(theta[1][j]) * math.sin(
            theta[2][j])) * math.cos(theta[0][j])),
                  ((math.cos(theta[1][j]) * math.cos(theta[2][j]) - math.sin(theta[1][j]) * math.sin(
                      theta[2][j])) * math.sin(theta[0][j])),
                  (math.cos(theta[1][j]) * math.sin(theta[2][j]) + math.sin(theta[1][j]) * math.cos(theta[2][j])),
                  0,
                  math.sin(theta[0][j]),
                  -math.cos(theta[0][j]),
                  0,
                  0,
                  ((math.cos(theta[1][j]) * math.sin(theta[2][j]) + math.sin(theta[1][j]) * math.cos(
                      theta[2][j])) * math.cos(theta[0][j])),
                  ((math.cos(theta[1][j]) * math.sin(theta[2][j]) + math.sin(theta[1][j]) * math.cos(
                      theta[2][j])) * math.sin(theta[0][j])),
                  (math.sin(theta[1][j]) * math.sin(theta[2][j]) - math.cos(theta[1][j]) * math.cos(theta[2][j])),
                  0]

        print Q3Q2Q1
        R = np.multiply(Q3Q2Q1,rotation)

        for k in xrange(j, j + 4, 4):
            cosroot1 = math.acos(
                max(min(1 / -mu[4] * (-lam[3] * (mu[5] * R[1] * lam[5] * R[2]) * math.sin(theta[3][k]) +
                                      lam[3] * (mu[5] * R[5] + lam[5] * R[6]) * math.cos(theta[3][k]) + mu[3] * (
                                          mu[5] * R[8] + lam[5] * R[10])), 1), -1))
            cosroot2 = -cosroot1 + 2 * math.pi

            temp_root = 1 / mu[4] * (
                (mu[5] * R[1] + lam[5] * R[2]) * math.cos(theta[3][k]) + (mu[5] * R[4] + lam[5] * R[6]) * math.sin(
                    theta[3][k]))
            sinroot1 = math.asin(temp_root)
            if temp_root >= 0:
                sinroot1 = math.asin(temp_root)
                sinroot2 = math.pi - sinroot1
            else:
                sinroot1 += 2 * math.pi
                sinroot2 = 3 * math.pi - sinroot1

            if abs(cosroot1 - sinroot1) < root_tol or abs(cosroot1 - sinroot2) < root_tol:
                theta[4][k] = cosroot1
            else:
                theta[4][k] = cosroot2

            w[0] = R[0] * math.cos(theta[3][k]) + R[4] * math.sin(theta[3][k])
            w[1] = -lam[3] * (R[0] * math.sin(theta[3][k]) - R[4] * math.cos(theta[3][k])) + mu[3] * R[8]
            w[2] = mu[3] * (R[0] * math.sin(theta[3][k]) - R[4] * math.cos(theta[3][k])) + lam[3] * R[8]

            cosroot1 = math.acos(max(min(w[0] * math.cos(theta[4][k]) + w[1] * math.sin(theta[4][k]), 1), -1))
            cosroot2 = -cosroot1 + 2 * math.pi

            temp_root = -w[0] * lam[4] * math.sin(theta[4][k]) + w[1] * lam[4] * math.cos(theta[4][k]) + w[2] * mu[4]
            sinroot1 = math.asin(temp_root)
            if temp_root >= 0:
                sinroot2 = math.pi - sinroot1
            else:
                sinroot1 += 2 * math.pi
                sinroot2 = 3 * math.pi - sinroot1

            if abs(cosroot1 - sinroot1) < root_tol or abs(cosroot1 - sinroot2) < root_tol:
                theta[5][k] = cosroot1
            else:
                theta[5][k] = cosroot2

    for i in xrange(0, 3):
        for j in xrange(0, 4):
            theta[i][j + 4] = theta[i][j]

    for i in xrange(0, 6):
        for j in xrange(0, 8):
            if theta[i][j] - angletol <= jointRanges[i][1]:
                theta[i][j] += 2 * math.pi
            elif theta[i][j] - angletol >= jointRanges[i][2]:
                theta[i][j] -= 2 * math.pi

    return theta


def lower_magnitude(a, b):
    return a if (a * a - b * b >= 0) else b


def possible_solution(theta):
    num_solutions = 0
    j_angle = []
    for i in xrange(0, 6):
        j_angle[i] = []
        for j in xrange(0, 8):
            j_angle[i][j] = 0

    for i in xrange(0, 8):
        valid_set = False
        for k in xrange(0, 3):
            if theta[k][i] > jointRanges[k][1] or (theta[k][i] < jointRanges[k][0]):
                valid_set = True

        if not valid_set:
            num_solutions += 1
            j_angle[j][num_solutions] = theta[j][i]

    return j_angle, num_solutions


def closest_solution(theta, num_solutions, current_position):
    distance_to_target = []
    for i in xrange(0, num_solutions):
        distance_to_target[i] = 0
        for j in xrange(0, 6):
            distance_to_target[i] = distance_to_target[i] + abs(theta[i][j] - current_position[j])

    mi = 0
    m = distance_to_target[mi]
    for i, value in enumerate(distance_to_target):  # change to index, value pairs
        if value < m:
            mi = i
            m = value
    return [theta[0][mi], theta[1][mi], theta[2][mi], theta[3][mi], theta[4][mi], theta[5][mi]]
