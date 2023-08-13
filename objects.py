import pygame
import constants
import calc
import physics
from physics import Thrust
from numpy.linalg import norm
import numpy as np


class Object:

    def __init__(self, id, mass, CoM, angle, colour, hasGravity, canCollide, checkCollisions, fixed, root=None):

        self.id = id
        self.screen = constants.screen
        self.mass = mass
        self.CoM = CoM
        self.angle = angle
        self.colour = colour
        self.defaultColour = colour

        self.root = root
        if self.root == None:
            self.root = self.CoM

        self.hasGravity = hasGravity
        self.checkCollisions = checkCollisions
        self.canCollide = canCollide
        self.fixed = fixed

        self.velocity = [0.0, 0.0]
        self.angularVelocity = 0.0
        self.nextVelocity = [0.0, 0.0]
        self.nextAngularVelocity = 0.0

        self.thrusts = []
        self.torques = []

        self.previousDistances = {}
        self.inContactWith = set()
        self.isCollided = False

        self.parent = None


    def getForces(self):
        return None


    def reset(self):
        self.thrusts = []
        self.torques = []


class Ball(Object):

    def __init__(self, id, CoM, radius, mass, colour, angle=0.0, hasGravity=True, canCollide=True, checkCollisions=True, calcPhysics=True, fixed=False):

        super().__init__(id=id, mass=mass, CoM=CoM, angle=angle, colour=colour, hasGravity=hasGravity, canCollide=canCollide, checkCollisions=checkCollisions, fixed=fixed)
        self.radius = radius
        self.radiusPixels = self.radius * constants.pixelsPerMetre
        self.MoI = physics.momentOfInertia(self, "circle")
        self.calcPhysics = calcPhysics

    def describe(self):
        print(f"CIRCLE -- id: {self.id}, centre: {self.CoM}, v: {self.velocity}, angle: {self.angle}, angleV: {self.angularVelocity}") 

    def draw(self):

        centrePixels = calc.metresToPixels(self.CoM)
        pygame.draw.circle(self.screen, self.colour, centrePixels, self.radiusPixels)

    def update(self):

        if self.calcPhysics:

            physics.applyForces(self)

            self.CoM[0] += self.velocity[0] * constants.TIME_STEP
            self.CoM[1] += self.velocity[1] * constants.TIME_STEP
            self.angle += self.angularVelocity * constants.TIME_STEP


class RigidBody(Object):

    def __init__(self, id, CoM, mass, colour, root=None, angle=0.0, hasGravity=True, canCollide=True, checkCollisions=True, calcPhysics=True, fixed=False):

        super().__init__(id=id, mass=mass, root=root, CoM=CoM, angle=angle, colour=colour, hasGravity=hasGravity, canCollide=canCollide, checkCollisions=checkCollisions, fixed=fixed)
        self.calcPhysics = calcPhysics

    def update(self):

        if self.calcPhysics:

            physics.applyForces(self)

            self.CoM[0] += self.velocity[0] * constants.TIME_STEP
            self.CoM[1] += self.velocity[1] * constants.TIME_STEP
            self.angle -= self.angularVelocity * constants.TIME_STEP


class Wall(RigidBody):

    def __init__(self, id, colour, start, end, width, hasGravity=False, canCollide=True, checkCollisions=False, fixed=True):

        super().__init__(id=id, mass=float('inf'), CoM=[0, 0], angle=0, colour=colour, hasGravity=hasGravity, canCollide=canCollide, checkCollisions=checkCollisions, calcPhysics=False, fixed=fixed)
        self.start = start
        self.end = end
        self.width = width

        self.startPixels = calc.metresToPixels(self.start)
        self.endPixels = calc.metresToPixels(self.end)
        self.widthPixels = calc.metreToPixels(self.width)

        self.MoI = float('inf')

    def draw(self):

        pygame.draw.line(self.screen, self.colour, self.startPixels, self.endPixels, round(self.widthPixels))


class Line(RigidBody):

    def __init__(self, id, mass, colour, width, root, rootToStart, rootToEnd, hasGravity=True, canCollide=True, checkCollisions=True, calcPhysics=True, fixed=False):
    
        self.start = calc.addVectors(root, rootToStart)
        self.end = calc.addVectors(root, rootToEnd)

        CoM = root      # when one is updated, so is the other
        angle = 0

        self.width = width
        self.length = norm(calc.getLineVector([self.start, self.end]))

        super().__init__(id=id, mass=mass, root=root, CoM=CoM, angle=angle, colour=colour, hasGravity=hasGravity, canCollide=canCollide, checkCollisions=checkCollisions, calcPhysics=calcPhysics, fixed=fixed)

        self.MoI = physics.momentOfInertia(self, "rod")
        self.rootToStart = rootToStart
        self.rootToEnd = rootToEnd

    def update(self):

        # put this in parent?
        if self.parent == None:
            super().update()
        else:
            """
            for torque in self.torques:
                physics.applyTorque(self, torque)
            """
            self.updateParent()

        self.start = calc.addVectors(self.root, calc.rotateVector(self.rootToStart, self.angle))
        self.end = calc.addVectors(self.root, calc.rotateVector(self.rootToEnd, self.angle))


    def updateParent(self):

        self.parent.velocity = self.velocity
        self.parent.angularVelocity = self.angularVelocity

        # need to get parent to update children?

        
    def draw(self):

        self.startPixels = calc.metresToPixels(self.start)
        self.endPixels = calc.metresToPixels(self.end)
        self.widthPixels = calc.metreToPixels(self.width)

        pygame.draw.line(self.screen, self.colour, self.startPixels, self.endPixels, round(self.widthPixels))
        pygame.draw.circle(self.screen, self.colour, calc.metresToPixels(self.root), self.widthPixels + 1)


class Spring(Object):

    def __init__(self, id, objectA, objectB, strength, colour, canCollide=False):

        super().__init__(id=id, mass=0, CoM=[0,0], angle=0, colour=colour, hasGravity=False, canCollide=canCollide, checkCollisions=False, fixed=False)
        self.objectA = objectA
        self.objectB = objectB
        self.strength = strength
        self.widthPixels = 15

    def getForces(self):
        length = norm(calc.getLineVector([self.objectA.CoM, self.objectB.CoM]))
        springForce = physics.springForce(self.strength, length)

        thrustAVector = calc.getLineVector([self.objectA.CoM, self.objectB.CoM])
        thrustAVector = calc.scaleVector(thrustAVector, springForce)

        thrustBVector = calc.getLineVector([self.objectB.CoM, self.objectA.CoM])
        thrustBVector = calc.scaleVector(thrustBVector, springForce)

        self.objectA.thrusts.append(physics.Thrust(
            thrustAVector, [0,0]
        ))

        self.objectB.thrusts.append(physics.Thrust(
            thrustBVector, [0,0]
        ))

    def update(self):
        return None

    def draw(self):
        self.startPixels = calc.metresToPixels(self.objectA.CoM)
        self.endPixels = calc.metresToPixels(self.objectB.CoM)

        pygame.draw.line(self.screen, self.colour, self.startPixels, self.endPixels, round(self.widthPixels))


class System(Object):

    def __init__(self, id, objects, root, CoM=None, angle=0, hasGravity=True, calcPhysics=True, fixed=True, sumMoI=True):

        # better way to set these manually
        mass = 0
        self.MoI = 0
        for object in objects:
            mass += object.mass
            if sumMoI:
                self.MoI += object.MoI
            else:
                # simple approximation of MoI - for lines only
                self.MoI += object.mass * norm(calc.getLineVector([root, calc.lineCentre(
                    [calc.addVectors(root, object.rootToStart),
                    calc.addVectors(root, object.rootToEnd)]
                )]))

        if CoM == None:
            # calculate CoM here
            CoM = [0, 0]

        self.objects = objects
        self.fixed = fixed
        self.calcPhysics = calcPhysics

        super().__init__(id=id, mass=mass, root=root, CoM=CoM, angle=angle, colour="white", hasGravity=hasGravity, canCollide=False, checkCollisions=False, fixed=fixed)

        # if self.fixed:
        self.pivotCoMvector = calc.getLineVector([self.root, self.CoM])
        self.pivotRadius = norm(self.pivotCoMvector)

        # each object has same MoI and mass as parent?
        for object in self.objects:
            object.mass = self.mass
            object.MoI = self.MoI
            object.parent = self
            object.fixed = self.fixed

    def describe(self):
        print(f"SYSTEM -- id: {self.id}, centre: {self.CoM}, v: {self.velocity}, angle: {self.angle}, angleV: {self.angularVelocity}, objects: {self.objects}") 


    def getForces(self):

        self.torques.append(physics.swingTorque(self))

        # need to put torques on children for proper calculations in Contact
        for object in self.objects:
            object.torques = self.torques

    def update(self):

        # move up object model?
        if not(self.fixed):
            physics.applyForces(self)
            self.CoM[0] += self.velocity[0] * constants.TIME_STEP
            self.CoM[1] += self.velocity[1] * constants.TIME_STEP

            self.root = self.CoM
            self.angle -= self.angularVelocity * constants.TIME_STEP

        elif self.fixed:

            if self.calcPhysics:

                physics.applyForces(self)

                self.angle -= self.angularVelocity * constants.TIME_STEP
                self.updateCoM()
                # if moving the object root, do this here

        for object in self.objects:
            object.root = self.root
            object.CoM = self.CoM
            object.angle = self.angle
            object.velocity = self.velocity
            object.angularVelocity = self.angularVelocity
            object.update()

        return None

    def updateCoM(self):

        # rotate CoM around pivot
        newVector = calc.rotateVector(self.pivotCoMvector, self.angle)
        self.CoM = [self.root[0] + newVector[0], self.root[1] + newVector[1]]

    def draw(self):

        return None


# need different centres and centre of masses?
# do not draw in relation to CoM!
# need to be able to move the centre without moving the CoM


    



    
        


