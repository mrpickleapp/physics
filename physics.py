import pygame
import numpy as np
from numpy import dot, cross, matmul
from numpy.linalg import norm
import constants
import calc
from calc import getLineVector, scalarMult, scalarDiv, squareVector, addVectors, subVectors, scaleVector, normaliseVector, theta
from scipy.integrate import odeint

# ---------------------------------------------------------------

class Thrust():

    def __init__(self, vector, point):
        self.vector = vector
        self.point = point

# optimised
def applyForces(object):

    def rb(y, t, g, m, thrusts, torques, I, k):
    
        vx, vy, theta = y

        # operate on list of forces
        if object.fixed == False:
            vx = (sum([T.vector[0] for T in thrusts]) - k * vx) / m
            vy = (sum([T.vector[1] for T in thrusts]) - m * g - k * vy) / m
        theta = (sum([np.cross(T.point, T.vector) for T in thrusts]) + sum(torques) - k * theta) / I

        dydt = [vx, vy, theta]
        
        return dydt

    y0 = [object.velocity[0], object.velocity[1], object.angularVelocity]

    timeStep = constants.TIME_STEP

    t = [0, timeStep, timeStep]

    sol = odeint(rb, y0, t, args=(
        constants.GRAVITY if object.hasGravity else 0,
        object.mass,
        object.thrusts,
        object.torques,
        object.MoI,
        constants.FRICTION
    ))

    if object.fixed == False:
        object.velocity[0] = sol[1][0]
        object.velocity[1] = sol[1][1]
    object.angularVelocity = sol[1][2]
    """
    object.nextVelocity[0] = sol[2][0]
    object.nextVelocity[1] = sol[2][1]
    object.nextAngularVelocity = sol[2][2]
    """


def resultOfForces(object):

    def rb(y, t, g, m, thrusts, torques, I, k):
    
        vx, vy, theta = y

        # operate on list of forces
        if object.fixed == False:
            vx = (sum([T.vector[0] for T in thrusts]) - k * vx) / m
            vy = (sum([T.vector[1] for T in thrusts]) - m * g - k * vy) / m
        theta = (sum([np.cross(T.point, T.vector) for T in thrusts]) + sum(torques) - k * theta) / I

        dydt = [vx, vy, theta]
        
        return dydt

    y0 = [object.velocity[0], object.velocity[1], object.angularVelocity]

    timeStep = constants.TIME_STEP

    t = [0, timeStep]

    sol = odeint(rb, y0, t, args=(
        constants.GRAVITY if object.hasGravity else 0,
        object.mass,
        object.thrusts,
        object.torques,
        object.MoI,
        constants.FRICTION
    ))

    forces = {}

    forces["velocity"] = [sol[1][0], sol[1][1]]
    forces["angularVelocity"] = sol[1][2]

    return forces


# optimised
def circleCircleCollision(objectA, objectB, intercept, distance):

    n = [intercept[0] / distance, intercept[1] / distance]
    p = (2 * (dot(objectA.velocity, n) - dot(objectB.velocity, n))) / (objectA.mass + objectB.mass)

    pma = p * objectA.mass
    pmb = p * objectB.mass

    elasticity = constants.ELASTICITY

    objectA.velocity = [objectA.velocity[0] - pmb * n[0] * elasticity, objectA.velocity[1] - pmb * n[1] * elasticity]
    objectB.velocity = [objectB.velocity[0] + pma * n[0] * elasticity, objectB.velocity[1] + pma * n[1] * elasticity]


# optimised
def reflect(vector, surface):

    if (vector[0] == 0) & (vector[1] == 0):
        return vector

    if surface[0] == 0:
        vector[0] *= -1 * constants.ELASTICITY
        return vector

    if surface[1] == 0:
        vector[1] *= -1 * constants.ELASTICITY
        return vector

    m = calc.m(surface)
    m2 = m*2
    msq = m**2
    x = 1 / (1 + msq)

    # transformation matrix
    T = [
        [(1-msq) * x, m2 * x],
        [m2 * x, (msq-1) * x]
    ]

    return matmul([vector[0] * constants.ELASTICITY, vector[1] * constants.ELASTICITY], T)


# this version ignores angular velocity of objectA, assumed to be circle
# ball is always objectA
def impulseA(objectA, objectB, pointA, pointB):

    AcomToPoint = getLineVector([objectA.root, pointA])
    BcomToPoint = getLineVector([objectB.root, pointB])

    # velocity of point = translational velocity vector + rotational velocity vector
    pointBrotationalSpeed = norm(BcomToPoint) * objectB.angularVelocity
    perpRotationVector = calc.rotateVector(BcomToPoint, 270)
    pointBrotationVector = calc.scaleVector(perpRotationVector, pointBrotationalSpeed)

    relPointV = subVectors(
        objectA.velocity,
        addVectors(
            objectB.velocity, pointBrotationVector
        )
    )

    perpImpactVector = normaliseVector(getLineVector([pointB, pointA]))

    aTerm = squareVector(cross(AcomToPoint, perpImpactVector)) / objectA.MoI
    bTerm = squareVector(cross(BcomToPoint, perpImpactVector)) / objectB.MoI

    impulse = (
        dot(relPointV, perpImpactVector) * -(1 + constants.ELASTICITY) /
        (
            1 / objectA.mass + 1 / objectB.mass + aTerm + bTerm
        )
    )

    if (objectA.fixed == False) & (objectB.fixed == False):

        objectA.velocity = addVectors(objectA.velocity, scalarDiv(scalarMult(perpImpactVector, impulse), objectA.mass))
        objectB.velocity = addVectors(objectB.velocity, scalarDiv(scalarMult(perpImpactVector, -impulse), objectB.mass))

        # objectA.angularVelocity += cross(AcomToPoint, scalarMult(perpImpactVector, impulse)) / objectA.MoI
        objectB.angularVelocity -= cross(BcomToPoint, scalarMult(perpImpactVector, impulse)) / objectB.MoI


    elif (objectA.fixed == False) & (objectB.fixed == True):

        # objectA simply gets the impulse applied twice?
        # bit of trial and error but this seems to work
        objectA.velocity = addVectors(objectA.velocity, scalarDiv(scalarMult(perpImpactVector, impulse), objectA.mass))
        objectA.velocity = addVectors(objectA.velocity, scalarDiv(scalarMult(perpImpactVector, impulse), objectB.mass))

        # objectA.angularVelocity += cross(AcomToPoint, scalarMult(perpImpactVector, impulse)) / objectA.MoI
        objectB.angularVelocity -= cross(BcomToPoint, scalarMult(perpImpactVector, impulse)) / objectB.MoI

    if objectA.parent != None:
        objectA.updateParent()

    if objectB.parent != None:
        objectB.updateParent()


# this version includes angular velocities of both
def impulseB(objectA, objectB, pointA, pointB):

    AcomToPoint = getLineVector([objectA.root, pointA])
    BcomToPoint = getLineVector([objectB.root, pointB])

    # velocity of point = translational velocity vector + rotational velocity vector
    pointArotationalSpeed = norm(AcomToPoint) * objectA.angularVelocity
    perpRotationVector = calc.rotateVector(AcomToPoint, 270)
    pointArotationVector = calc.scaleVector(perpRotationVector, pointArotationalSpeed)

    pointBrotationalSpeed = norm(BcomToPoint) * objectB.angularVelocity
    perpRotationVector = calc.rotateVector(BcomToPoint, 270)
    pointBrotationVector = calc.scaleVector(perpRotationVector, pointBrotationalSpeed)

    relPointV = subVectors(
        addVectors(
            objectA.velocity, pointArotationVector
        ),
        addVectors(
            objectB.velocity, pointBrotationVector
        )
    )

    perpImpactVector = normaliseVector(getLineVector([pointB, pointA]))

    aTerm = squareVector(cross(AcomToPoint, perpImpactVector)) / objectA.MoI
    bTerm = squareVector(cross(BcomToPoint, perpImpactVector)) / objectB.MoI

    impulse = (
        dot(relPointV, perpImpactVector) * -(1 + constants.ELASTICITY) /
        (
            1 / objectA.mass + 1 / objectB.mass + aTerm + bTerm
        )
    )

    if (objectA.fixed == False) & (objectB.fixed == False):

        objectA.velocity = addVectors(objectA.velocity, scalarDiv(scalarMult(perpImpactVector, impulse), objectA.mass))
        objectB.velocity = addVectors(objectB.velocity, scalarDiv(scalarMult(perpImpactVector, -impulse), objectB.mass))

    elif (objectA.fixed == False) & (objectB.fixed == True):

        objectA.velocity = addVectors(objectA.velocity, scalarDiv(scalarMult(perpImpactVector, impulse), objectA.mass))
        objectA.velocity = addVectors(objectA.velocity, scalarDiv(scalarMult(perpImpactVector, impulse), objectB.mass))

    elif (objectA.fixed == True) & (objectB.fixed == False):

        objectB.velocity = addVectors(objectA.velocity, scalarDiv(scalarMult(perpImpactVector, impulse), objectA.mass))
        objectB.velocity = addVectors(objectA.velocity, scalarDiv(scalarMult(perpImpactVector, impulse), objectB.mass))

    objectA.angularVelocity += cross(AcomToPoint, scalarMult(perpImpactVector, impulse)) / objectA.MoI
    objectB.angularVelocity -= cross(BcomToPoint, scalarMult(perpImpactVector, impulse)) / objectB.MoI

    # for objects in system
    if objectA.parent != None:
        objectA.updateParent()

    if objectB.parent != None:
        objectB.updateParent()

    if constants.SHOW_FORCES:
        pygame.draw.line(constants.screen, "red", calc.metresToPixels(pointA), 
            calc.metresToPixels(calc.addVectors(pointB, perpImpactVector)), 
        4)
        pygame.draw.line(constants.screen, "red", calc.metresToPixels(pointB), 
            calc.metresToPixels(calc.subVectors(pointA, perpImpactVector)), 
        4)


# collision where objectB has infinite mass and is not moving
# ball is always objectA
def impulseInf(objectA, pointA, pointB):

    AcomToPoint = getLineVector([objectA.root, pointA])

    # velocity of point = translational velocity vector + rotational velocity vector
    pointArotationalSpeed = norm(AcomToPoint) * objectA.angularVelocity
    perpRotationVector = calc.rotateVector(AcomToPoint, 270)
    pointArotationVector = calc.scaleVector(perpRotationVector, pointArotationalSpeed)

    relPointV = addVectors(objectA.velocity, pointArotationVector)

    perpImpactVector = normaliseVector(getLineVector([pointB, pointA]))

    aTerm = squareVector(cross(AcomToPoint, perpImpactVector)) / objectA.MoI

    impulse = (
        dot(relPointV, perpImpactVector) * -(1 + constants.ELASTICITY) /
        (1 / objectA.mass + aTerm)
    )

    objectA.velocity = addVectors(objectA.velocity, scalarDiv(scalarMult(perpImpactVector, impulse), objectA.mass))
    objectA.angularVelocity += cross(AcomToPoint, scalarMult(perpImpactVector, impulse)) / objectA.MoI

    if objectA.parent != None:
        objectA.updateParent()


def swingTorque(object):

    if object.pivotRadius == 0:
        return 0

    pivotToCoM = getLineVector([object.root, object.CoM])
    angle = np.radians(270) - theta(pivotToCoM, [1, 0])
    force = constants.GRAVITY * np.sin(angle)
    torque = object.pivotRadius * force

    return torque


def applyTorque(object, torque):

    acc = (torque / object.MoI) * constants.TIME_STEP
    object.angularVelocity -= acc



# ---------------------------------------------------------------


# floors tiny velocities
# entropic decay for all velocities
def velocityFilter(v):

    v[0] *= constants.GENERAL_DAMPING_F
    v[1] *= constants.GENERAL_DAMPING_F

    if abs(v[0]) < constants.VELOCITY_FLOOR:
        v[0] = 0
    if abs(v[1]) < constants.VELOCITY_FLOOR:
        v[1] = 0

    return v

# used for proximity damping
# not time-step adjusted
def dampVelocity(v, type):

    if type == "exponential":

        v[0] = expDamp(v[0])
        v[1] = expDamp(v[1])

    return v

def expDamp(f):

    m = -1 if f < 0 else 1

    f = m* (abs(f) * (1-1/np.exp(abs(f))))

    return f


# takes list of forces, calculates agg force
def netForce(forces):

    netF = [0, 0]

    for force in forces:
        netF[0] += force[0]
        netF[1] += force[1]

    return netF


def acceleration(netF, mass):

    return [netF[0] / mass, netF[1] / mass]


def initForces(object):

    object.forces.extend([[0, -constants.GRAVITY * object.mass]])


def resetForces(object):

    object.forces.clear()


def calcAcc(object):
    r"""
    Basic Acceleration for an object in space
    Can only be used for gravity or other constant forces
    User input falls into this category because it is applied every sim iteration per render frame
    """

     # filter velocity
    object.velocity = velocityFilter(object.velocity)

    # net forces
    netF = netForce(object.forces)

    # acceleration
    acc = acceleration(netF, object.mass)

    # apply acceleration to velocity
    object.velocity[0] += acc[0]     # these still causing trouble? - but maybe ok if just used for gravitational?
    object.velocity[1] += acc[1]

    # apply velocity to object
    object.root[0] += object.velocity[0] / constants.SIM_MULTIPLIER
    object.root[1] += object.velocity[1] / constants.SIM_MULTIPLIER


def calcAngle(object):

    object.angle += object.angularSpeed / constants.SIM_MULTIPLIER
    object.coords = calc.lineEnds(object.root, object.angle, object.length)


def momentOfInertia(object, type):

    # circle
    if type == "circle":
        return np.pi * object.radius ** 4 / 4

    # axis at the centre
    if type == "rod":
        return (1/12) * object.mass * object.length ** 2 

    # axis at one end
    if type == "pendulum":
        return (1/3) * object.mass * object.length ** 2 


def angularMomentum(object):

    return object.momentOfInertia * object.angularSpeed


def angularMomentumPoint(mass, velocity, distance, th):

    return mass * velocity * distance * np.sin(th)


def angularSpeedFromMomentum(object):

    return object.angularMomentum / object.momentOfInertia


def springForce(strength, length):

    return strength * length**2
