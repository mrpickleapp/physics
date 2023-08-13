from random import randint, random


# ------------------------------------------------------------

screen = None

screenMetres = [3.2, 2]         # metres
screenPixels = [1280, 800]      # pixels
pixelsPerMetre = 1280 / 3.2     # pixels

TICK = 60
SIM_MULTIPLIER = 2             # how many times the simulation runs per render tick
SPEED = 0.5                    # 1 = 100%

TIME_STEP = 1 / TICK / SIM_MULTIPLIER * SPEED       # seconds

def setTimeStep():
    return 1 / TICK / SIM_MULTIPLIER * SPEED

timeStamp = 0                   # seconds

GRAVITY = 9.81 # m/s/s
FRICTION = 0
ELASTICITY = 0.5
INPUT_FORCE = 2
SPRING_FORCE = 3

DEBUG_MODE = False
DEBUG_PANEL = False
SHOW_FORCES = False

GENERAL_DAMPING = 0.25 # normally 0.001

APPROACH_FACTOR_FLOOR = -0.01

# How many radii to add to length when checking for perp collision with line
COLLISION_PRUNE_PERP_F = 1   # 0.75 is safe?
# When checking for end point collision
COLLISION_PRUNE_END_F = 2    # 1.75 is safe

def RANDOM_COLOUR():
    return [randint(0, 255), randint(0, 255), randint(0, 255)]

def RANDOM_RADIUS():
    return random() / 5


simIterationFunctions = []

outputFileName = None

# ------------------------------------------------------------

contactForces = []


WALL_WIDTH = 10






LOSS_FACTOR = 1.0  # normally 0.98


GENERAL_DAMPING_F = 1 - GENERAL_DAMPING / SIM_MULTIPLIER

VELOCITY_FLOOR = 1e-2





