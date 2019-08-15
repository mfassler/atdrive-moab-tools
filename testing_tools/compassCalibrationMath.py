#!/usr/bin/env python3

import sys
if sys.version_info[0] < 3:
    raise Exception("Must be using Python 3")

import pickle
import numpy as np
import matplotlib.pyplot as plt


f = open('exampleCompassValues.pkl', 'rb')
_vals = pickle.load(f)
f.close()
vals = np.array(_vals)


# https://stackoverflow.com/questions/47873759/how-to-fit-a-2d-ellipse-to-given-points
_X = vals[:, 0]
_Y = vals[:, 1]

X = _X.reshape((_X.shape[0], 1))
Y = _Y.reshape((_Y.shape[0], 1))

# Formulate and solve the least squares problem ||Ax - b ||^2
A = np.hstack([X**2, X * Y, Y**2, X, Y])
b = np.ones_like(X)
x = np.linalg.lstsq(A, b, rcond=None)[0].squeeze()


def get_canonical_params(x):
    # from https://en.wikipedia.org/wiki/Ellipse, "General ellipse"
    A = x[0]
    B = x[1]
    C = x[2]
    D = x[3]
    E = x[4]
    F = -1.0
    _denominator = (B**2 - 4*A*C)
    _part1 = 2*(A*E**2 + C*D**2 - B*D*E + _denominator * F)
    _part2 = (A+C)
    _part3 = np.sqrt((A-C)**2 + B**2)
    a = np.sqrt(_part1 * (_part2 + _part3)) / _denominator
    b = np.sqrt(_part1 * (_part2 - _part3)) / _denominator
    x0 = (2*C*D - B*E) / _denominator
    y0 = (2*A*E - B*D) / _denominator
    if B == 0:
        if A < C:
            theta = 0
        else:
            theta = np.radians(90.0)
    else:
        theta = np.arctan((C - A - np.sqrt((A-C)**2 + B**2)) / B)

    # TODO? FIXME?  Is it normal for a and b to come out negative?
    return np.abs(a), np.abs(b), x0, y0, theta


def make_xfrm_matrix(a, b, x0, y0, theta):
    # Scaling:
    xfrm1 = np.array([[_a,  0, 0],
                      [ 0, _b, 0],
                      [ 0,  0, 1]])

    # Rotation and offset:
    xfrm2 = np.array([[ np.cos(theta), -np.sin(theta), x0],
                      [ np.sin(theta),  np.cos(theta), y0],
                      [      0,             0,          1]])

    xfrm = xfrm2.dot(xfrm1)
    return xfrm


_a, _b, x0, y0, theta = get_canonical_params(x)
xfrm = make_xfrm_matrix(_a, _b, x0, y0, theta)

print("x_center: %.04f,  y_center: %.04f" % (x0, y0))
print("x_scale: %.04f,  y_scale: %.04f" % (_a, _b))
print("rotation: %.01f degrees" % (np.degrees(theta)))

t = np.linspace(0, 2*np.pi, 100)
unit_circle = np.stack(( np.cos(t), np.sin(t), np.ones(len(t)) )).T

vector_up = np.vstack((np.zeros(10), np.linspace(0, 1, 10), np.ones(10))).T
vector_right = np.vstack((np.linspace(0, 1, 10), np.zeros(10), np.ones(10))).T


recon = np.dot(xfrm, unit_circle.T).T
new_up = np.dot(xfrm, vector_up.T).T
new_right = np.dot(xfrm, vector_right.T).T

plt.plot( [x0], [y0], 'b.')
plt.plot(vals[:, 0], vals[:, 1], 'r.')
plt.plot(recon[:, 0], recon[:, 1], 'b')
plt.plot(new_up[:, 0], new_up[:, 1], 'y')
plt.plot(new_right[:, 0], new_right[:, 1], 'g')

plt.axis('equal')
plt.show()


