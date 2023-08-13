import constants
import numpy as np

halfScreenPixelsX = constants.screenPixels[0] / 2
halfScreenPixelsY = constants.screenPixels[1] / 2

halfScreenPixelsXneg = constants.screenPixels[0] / 2

r90 = np.pi / 2
T90 = [
    [0, -1],
    [1, 0]
]
T270 = [
    [0, 1],
    [-1, 0]
]

