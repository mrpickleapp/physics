import numpy as np
from numpy import arccos, array, matmul
from numpy.linalg import norm
import constants
import values


def metreToPixels(m):
    r"""Takes single m and converts to pixels
    """
    return m * constants.pixelsPerMetre


def metresToPixels(m):
    r"""Takes metres in [x, y] and converts to pixels, with [0, 0] at centre of screen
    - inverts y axis for drawing
    """

    return [
        values.halfScreenPixelsX + metreToPixels(m[0]),
        values.halfScreenPixelsY - metreToPixels(m[1])
    ]

def pixelToMetre(p):
    return p / constants.pixelsPerMetre

def pixelsToMetres(p):
    r"""Takes pixels in [x, y] and converts to metres, with [0, 0] at centre of screen
    - inverts y axis for drawing
    """
    return [
        -constants.screenMetres[0] / 2 + pixelToMetre(p[0]),
        constants.screenMetres[1] / 2 - pixelToMetre(p[1])
    ]


def isMoving(velocity):
    if (velocity[0] != 0) | (velocity[1] != 0):
        return True
    else:
        return False


def addVectors(a, b):

    return [a[0] + b[0], a[1] + b[1]]

def subVectors(a, b):

    return [a[0] - b[0], a[1] - b[1]]

def scalarMult(vector, scalar):

    return [vector[0] * scalar, vector[1] * scalar]

def scalarDiv(vector, scalar):

    return [vector[0] / scalar, vector[1] / scalar]

def squareVector(vector):

    return np.dot(vector, vector)


def getLineVector(line):
    r"""[x, y] vector of a line described by two points
    :param line [[Ax, Ay], [Bx, By]]
    :return [x, y]
    """

    A = line[0]
    B = line[1]

    vector = [B[0] - A[0], B[1] - A[1]]

    return vector


def rotateVector(vector, th):

    if th == 90:

        return matmul(vector, values.T90)

    if th == 270:

        return matmul(vector, values.T270)

    else:

        a = np.cos(th)
        b = np.sin(th)

        T = [
            [a, -b],
            [b, a]
        ]

        return matmul(vector, T)


def vectorFromAngle(th):

    return [np.cos(th), np.sin(th)]


def theta(v1, v2): 
    r"""Angle between two vectors, in radians, measured from vector roots
    :param v1 [x, y]
    :param v1 [x, y]
    :return angle (radians)
    """

    v1 = array(v1)
    v2 = array(v2)

    # sometimes math inaccuracy results is values slightly outside -1 to 1
    # this creates arccos error
    
    return arccos(v1.dot(v2)/(norm(v1)*norm(v2)))


def scaleVector(vector, length):

    n = norm(vector)

    f = (length / n) if n != 0 else 0

    return [f * i for i in vector]


def normaliseVector(vector):

    n = norm(vector)

    return [v / n for v in vector]


def lineCentre(coords):

    halfX = (coords[0][0] - coords[1][0]) / 2
    halfY = (coords[0][1] - coords[1][1]) / 2

    return [coords[0][0] - halfX, coords[0][1] - halfY]


def lineEnds(centre, th, length):

    coords = [None, None]

    coords = [centre.copy(), centre.copy()]

    A = [length / 2, 0]

    a = np.cos(th)
    b = np.sin(th)

    rotation = [
        [a, b],
        [-b, a]
    ]

    offset = np.matmul(rotation, A)

    coords[0][0] -= offset[0]
    coords[0][1] -= offset[1]
    coords[1][0] += offset[0]
    coords[1][1] += offset[1]

    return coords


def pivotEnd(pivot, th, length):

    end = pivot.copy()
    A = [length, 0]

    a = np.cos(th)
    b = np.sin(th)

    rotation = [
        [a, b],
        [-b, a]
    ]

    offset = np.matmul(rotation, A)

    end[0] += offset[0]
    end[1] += offset[1]

    return end


def circleArea(r):
    return np.pi * r**2


def m(vector):

    return vector[1] / vector[0]