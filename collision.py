import calc
from calc import scaleVector, normaliseVector, theta, getLineVector, isMoving, lineCentre, rotateVector, addVectors
from numpy import matmul, cross, dot
from numpy.linalg import norm
import constants
from objects import Wall, Ball
import contacts
from contacts import Contact
from physics import circleCircleCollision, reflect
import physics
import math


# iterates through dictionary and looks for collisions
def calcCollisions(objects):

    # list of objects that have been processed - don't need to save pairs? just one half
    handledColliders = []

    # only check collisions for objects that can collide
    # this means ignoring fixed scenery objects
    # may cause problems later?
    # only check collisions for moving objects

    for objectId in objects:

        object = objects[objectId]

        if object.checkCollisions:

            for otherObjectId in objects:

                otherObject = objects[otherObjectId]

                if (otherObject.canCollide) & (otherObjectId != objectId) & (otherObjectId not in handledColliders):   # object cannot collide with self

                    cont = True

                    if (object.parent != None) & (otherObject.parent != None):
                        if object.parent.id == otherObject.parent.id:
                            cont = False

                    # need to do this better
                    if cont:

                        if object.__class__.__name__ == "Ball":

                            if otherObject.__class__.__name__ == "Wall":

                                circleLineDetection(object, otherObject, onlyReflect=True)

                            elif otherObject.__class__.__name__ == "Ball":

                                circleCircleDetection(object, otherObject)

                            elif otherObject.__class__.__name__ == "Line":

                                circleLineDetection(object, otherObject, onlyReflect=False)

                        elif object.__class__.__name__ == "Line":

                            if otherObject.__class__.__name__ == "Ball":

                                circleLineDetection(otherObject, object, onlyReflect=False)

                            elif otherObject.__class__.__name__ == "Wall":

                                lineLineDetection(object, otherObject, onlyReflect=True, end=object.start)
                                lineLineDetection(object, otherObject, onlyReflect=True, end=object.end)

                            elif otherObject.__class__.__name__ == "Line":

                                if lineLineDetection(object, otherObject, onlyReflect=False, end=object.start) != True:
                                    if lineLineDetection(object, otherObject, onlyReflect=False, end=object.end) != True:
                                        if lineLineDetection(otherObject, object, onlyReflect=False, end=otherObject.start) != True:
                                            lineLineDetection(otherObject, object, onlyReflect=False, end=otherObject.end)

            handledColliders.append(objectId)


def lineLineDetection(lineA, lineB, onlyReflect, end):

    line_a = getLineVector([end, lineB.end])
    line_b = getLineVector([lineB.start, end])
    line_c = getLineVector([lineB.start, lineB.end])

    length_a = norm(line_a)
    length_b = norm(line_b)
    length_c = norm(line_c)

    # check closeness
    if ((length_a + length_b) > (length_c + constants.COLLISION_PRUNE_PERP_F * 0.02)) & ((length_a > constants.COLLISION_PRUNE_END_F * 0.02) & (length_b > constants.COLLISION_PRUNE_END_F * 0.02)):
        return False

    # calc angles
    angleA = theta(line_c, line_b)
    angleC = theta(line_b, getLineVector([lineB.end, end]))
    angleB = theta(line_a, line_c)

    shortestDistance = None
    intersect = None

    # if A or B is over 90, then use lines b or a
    # collision will occur on one end
    if angleA >= 1.5708:

        if constants.DEBUG_MODE:
            Line_b = Wall(id=-1, colour="gray", start=lineB.start, end=end, width=0.01)
            Line_b.draw()

        shortestDistance = length_b
        intersect = lineB.start

    # collision will occur on the other end
    elif angleB >= 1.5708:

        if constants.DEBUG_MODE:
            Line_a = Wall(id=-1, colour="gray", start=lineB.end, end=end, width=0.01)
            Line_a.draw()

        shortestDistance = length_a
        intersect = lineB.end

    # collision will occur between ends
    else:

        intersect = calc.addVectors(lineB.start, calc.scaleVector(line_c, dot(line_b, line_c) / norm(line_c)))
        shortestDistance = norm(calc.getLineVector([intersect, end]))
        
        if constants.DEBUG_MODE:
            Line_h = Wall(id=-1, colour="gray", start=intersect, end=end, width=0.01)
            Line_h.draw()

    # check convergence
    # converging = checkConvergence(lineA, lineB, shortestDistance) 

    # handle collision

    pointA = end
    pointB = intersect

    # TESTING
    AcomToPoint = getLineVector([lineA.root, pointA])
    BcomToPoint = getLineVector([lineB.root, pointB])

    # velocity of point = translational velocity vector + rotational velocity vector
    pointArotationalSpeed = norm(AcomToPoint) * lineA.angularVelocity
    perpRotationVector = calc.rotateVector(AcomToPoint, 270)
    pointArotationVector = calc.scaleVector(perpRotationVector, pointArotationalSpeed)

    pointBrotationalSpeed = norm(BcomToPoint) * lineB.angularVelocity
    perpRotationVector = calc.rotateVector(BcomToPoint, 270)
    pointBrotationVector = calc.scaleVector(perpRotationVector, pointBrotationalSpeed)

    relPointV = calc.subVectors(
        addVectors(
            lineA.velocity, pointArotationVector
        ),
        addVectors(
            lineB.velocity, pointBrotationVector
        )
    )

    perpImpactVector = normaliseVector(getLineVector([pointB, pointA]))
    approachFactor = dot(relPointV, perpImpactVector)

    converging = True if approachFactor < 0 else False

    collision = False
    contact = False

    if converging & (shortestDistance <= (lineA.width + lineB.width) / 2):

        if approachFactor < constants.APPROACH_FACTOR_FLOOR:
            collision = True

    if shortestDistance <= (lineA.width + lineB.width) / 2:

        contact = True
    

    if constants.DEBUG_MODE:
        pointAMarker = Ball(id=-1, CoM=pointA, radius=0.03, mass=0, colour="orange")
        pointAMarker.draw()

        pointBMarker = Ball(id=-1, CoM=pointB, radius=0.03, mass=0, colour="orange")
        pointBMarker.draw()

        Line_p = Wall(id=-1, colour="gray", start=pointB, end=pointA, width=0.01)
        Line_p.draw()


    if collision:

        if onlyReflect:

            # collision with fixed line
            physics.impulseInf(lineA, end, intersect)

        else:

            # collision with moving line
            physics.impulseB(lineA, lineB, pointA, pointB)

        return True


    elif contact:

        # handle contact force here
        # pretend it is just torque for the moment?
        # if this works, we know to create a new contact here for resolution later

        cont = Contact(
            objectA=lineA, objectB=lineB,
            pointA=pointA, pointB=pointB,
            AcomToPoint=AcomToPoint, BcomToPoint=BcomToPoint,
            perpImpactVector=perpImpactVector
        )

        contacts.contactsToResolve.append(cont)

        return True

    return False


def circleLineDetection(circle, line, onlyReflect):

    line_a = getLineVector([circle.CoM, line.end])
    line_b = getLineVector([line.start, circle.CoM])
    line_c = getLineVector([line.start, line.end])

    length_a = norm(line_a)
    length_b = norm(line_b)
    length_c = norm(line_c)

    # check closeness
    if ((length_a + length_b) > (length_c + constants.COLLISION_PRUNE_PERP_F * circle.radius)) & ((length_a > constants.COLLISION_PRUNE_END_F * circle.radius) & (length_b > constants.COLLISION_PRUNE_END_F * circle.radius)):
        return None

    # calc angles
    angleA = theta(line_c, line_b)
    angleC = theta(line_b, getLineVector([line.end, circle.CoM]))
    angleB = theta(line_a, line_c)

    shortestDistance = None
    intersect = None
    reflectionLine = None

    # if A or B is over 90, then use lines b or a
    # collision will occur on one end
    if angleA >= 1.5708:

        if constants.DEBUG_MODE:
            Line_b = Wall(id=-1, colour="gray", start=line.start, end=circle.CoM, width=0.01)
            Line_b.draw()

        shortestDistance = length_b
        intersect = line.start
        reflectionLine = rotateVector(line_b, 90)

    # collision will occur on the other end
    elif angleB >= 1.5708:

        if constants.DEBUG_MODE:
            Line_a = Wall(id=-1, colour="gray", start=line.end, end=circle.CoM, width=0.01)
            Line_a.draw()

        shortestDistance = length_a
        intersect = line.end
        reflectionLine = rotateVector(line_a, 90)

    # collision will occur between ends
    # can probs do this better with dot product
    else:

        intersect = calc.addVectors(line.start, calc.scaleVector(line_c, dot(line_b, line_c) / norm(line_c)))
        shortestDistance = norm(calc.getLineVector([intersect, circle.CoM]))
        reflectionLine = line_c

        reflectionLine = line_c
        
        if constants.DEBUG_MODE:
            Line_h = Wall(id=-1, colour="gray", start=intersect, end=circle.CoM, width=0.01)
            Line_h.draw()


    # check convergence
    # converging = checkConvergence(circle, line, shortestDistance)

    pointA = circle.CoM     # is this fine?
    pointB = intersect


    # get velocity of point B
    pointBvelocity = [0, 0]

    if line.calcPhysics == True:
        BcomToPoint = getLineVector([line.root, pointB])
        pointBrotationalSpeed = norm(BcomToPoint) * line.angularVelocity
        perpRotationVector = calc.rotateVector(BcomToPoint, 270)
        pointBrotationVector = calc.scaleVector(perpRotationVector, pointBrotationalSpeed)
        pointBvelocity = calc.addVectors(line.velocity, pointBrotationVector)

    perpImpactVector = normaliseVector(getLineVector([pointB, pointA]))

    relPointV = calc.subVectors(circle.velocity, pointBvelocity)

    approachFactor = dot(relPointV, perpImpactVector)

    converging = True if approachFactor < 0 else False

    collision = False
    contact = False

    if converging & (shortestDistance <= circle.radius + line.width / 2):

        if approachFactor < constants.APPROACH_FACTOR_FLOOR:
            collision = True

    if shortestDistance <= circle.radius + line.width / 2:

        contact = True

    if constants.DEBUG_MODE:
        pointAMarker = Ball(id=-1, CoM=pointA, radius=0.03, mass=0, colour="orange")
        pointAMarker.draw()

        pointBMarker = Ball(id=-1, CoM=pointB, radius=0.03, mass=0, colour="orange")
        pointBMarker.draw()

        Line_p = Wall(id=-1, colour="gray", start=pointB, end=circle.CoM, width=0.01)
        Line_p.draw()

    elif collision:

        if onlyReflect:
            circle.velocity = reflect(circle.velocity, reflectionLine)

        elif onlyReflect == False:

            physics.impulseA(circle, line, pointA, pointB)

    if contact:

        cont = Contact(
            objectA=circle, objectB=line,
            pointA=pointA, pointB=pointB,
            perpImpactVector=perpImpactVector,
            type=0
        )

        contacts.contactsToResolve.append(cont)

    
def circleCircleDetection(circleA, circleB):

    if constants.DEBUG_MODE:
        Line_a = Wall(id=-1, colour="gray", start=circleA.CoM, end=circleB.CoM, width=0.001)
        Line_a.draw()

    radSum = circleA.radius + circleB.radius

    intercept = getLineVector([circleA.CoM, circleB.CoM])
    distance = norm(intercept)

    perpImpactVector = normaliseVector(getLineVector([circleB.CoM, circleA.CoM]))
    approachFactor = dot(calc.subVectors(circleA.velocity, circleB.velocity), perpImpactVector)

    collision = False
    contact = False

    # converging = checkConvergence(circleA, circleB, distance)
    converging = True if approachFactor < 0 else False

    if converging & (distance <= radSum):

        if approachFactor < constants.APPROACH_FACTOR_FLOOR:
            collision = True

    if distance <= radSum:

        contact = True

    if collision:

        circleCircleCollision(circleA, circleB, intercept, distance)

    # using resting forces to make up for bad use of elasticity in physics
    if contact:

        cont = Contact(
            objectA=circleA, objectB=circleB,
            pointA=circleA.CoM, pointB=circleB.CoM,
            perpImpactVector=perpImpactVector
        )

        contacts.contactsToResolve.append(cont)


def checkConvergence(objectA, objectB, distance):

     # Test convergence
    previousDistance = objectA.previousDistances.get(objectB.id)

    if previousDistance == None:
        previousDistance = distance

    objectA.previousDistances[objectB.id] = distance
    objectB.previousDistances[objectA.id] = distance    # probs not necessesary because b will always be processessed after a

    return True if previousDistance > distance else False



