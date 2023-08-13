import pygame
import physics
import constants
import calc
import numpy as np
from numpy.linalg import norm


contactsToResolve = []

class Contact():

    def __init__(
        self, 
        objectA, objectB,
        pointA, pointB,
        AcomToPoint=None, BcomToPoint=None,
        perpImpactVector=None,
        type = None,
        pointArotationVector = None,
        pointBrotationVector = None,
        relPointV = None
        ):
        self.objectA = objectA
        self.objectB = objectB
        self.pointA = pointA
        self.pointB = pointB
        self.AcomToPoint = AcomToPoint
        self.BcomToPoint = BcomToPoint
        self.perpImpactVector = perpImpactVector
        self.type = type
        self.pointArotationVector = None,
        self.pointBrotationVector = None,
        self.relPointV = None

    def resolve(self):

        return None

        self.resolveGeneral()

        """

        if self.type == 0:
            self.resolveBallWall()
        
        elif self.type == 3:
            self.resolveLineLine()

        """
    
    def resolveGeneral(self):

        # print("CONTACT FORCES")
        # print("TIME", constants.timeStamp)

        # print("objectA", self.objectA.__class__.__name__)
        # print("objectB", self.objectB.__class__.__name__)

        pointAVector = [0, 0]   # how pointA is moving in space
        pointBVector = [0, 0]

        AcomToPoint = [0, 0]
        BcomToPoint = [0, 0]

        pointArotationalSpeed = 0
        pointBrotationalSpeed = 0

        if self.objectA.calcPhysics:
            AcomToPoint = calc.getLineVector([self.objectA.root, self.pointA])
            nextAVelocities = physics.resultOfForces(self.objectA)

            pointArotationVector = None

            # velocity of point = translational velocity vector + rotational velocity vector
            if (self.objectA.angularVelocity != 0) & (AcomToPoint[0] != 0) & (AcomToPoint[1] != 0):
                pointArotationalSpeed = norm(AcomToPoint) * nextAVelocities["angularVelocity"]
                perpRotationVector = calc.rotateVector(AcomToPoint, 270)
                pointArotationVector = calc.scaleVector(perpRotationVector, pointArotationalSpeed)
            else:
                pointArotationVector = [0, 0]

            pointAVector = calc.addVectors(nextAVelocities["velocity"], pointArotationVector)

        else:
            pointAVector = [0, 0]

        
        if self.objectB.calcPhysics:
            nextBVelocities = physics.resultOfForces(self.objectB)

            BcomToPoint = calc.getLineVector([self.objectB.root, self.pointB])

            pointBrotationVector = None

            if (self.objectB.angularVelocity != 0) & (BcomToPoint[0] != 0) & (BcomToPoint[1] != 0):
                pointBrotationalSpeed = norm(BcomToPoint) * nextBVelocities["angularVelocity"]
                perpRotationVector = calc.rotateVector(BcomToPoint, 270)
                pointBrotationVector = calc.scaleVector(perpRotationVector, pointBrotationalSpeed)
            else:
                pointBrotationVector = [0, 0]

            pointBVector = calc.addVectors(nextBVelocities["velocity"], pointBrotationVector)

        relPointV = calc.subVectors(
            pointAVector, pointBVector
        )

        perpImpactVector = calc.normaliseVector(calc.getLineVector([self.pointB, self.pointA]))
        aTerm = (calc.squareVector(np.cross(AcomToPoint, perpImpactVector)) / self.objectA.MoI) if self.objectA.calcPhysics else 0
        bTerm = (calc.squareVector(np.cross(BcomToPoint, perpImpactVector)) / self.objectB.MoI) if self.objectB.calcPhysics else 0

        closingSpeed = np.dot(relPointV, perpImpactVector)

        # objects are actually moving apart
        if closingSpeed >= 0:
            # print("objects not closing")
            return None

        force = (
            closingSpeed * -1 /
            (
                (1 / self.objectA.mass + aTerm) if self.objectA.calcPhysics else 0
                + (1 / self.objectB.mass + bTerm) if self.objectB.calcPhysics else 0
            )
        ) / constants.TIME_STEP

        # print("force", force)
        # print("perpImpactVector", self.perpImpactVector)

        # force to cancel closingVelocity / closing angular Momentum
        # does not yet handle angular momentum
        opposingForceVector = calc.scaleVector(self.perpImpactVector, force)

        # print("Opposing Force Vector", opposingForceVector)

        if self.objectA.calcPhysics == True:

            if self.objectA.fixed == False:

                self.objectA.thrusts.append(physics.Thrust(
                    vector=opposingForceVector, point=AcomToPoint
                ))

                if constants.SHOW_FORCES:
                    pygame.draw.line(constants.screen, "purple", calc.metresToPixels(self.pointA), 
                        calc.metresToPixels(calc.addVectors(self.pointA, opposingForceVector)), 
                    4)

            else:

                # object is rotating around fixed pivot
                # turn thrust into a torque
                # Just need to figure out how to sign the torque!

                """
                
                print("Opposing force vector", opposingForceVector)

                torque = norm(opposingForceVector) * norm(AcomToPoint)

                print("Torque A", torque)

                self.objectA.parent.torques.append(torque)

                """
                self.objectA.parent.thrusts.append(physics.Thrust(
                    vector=[opposingForceVector[0], opposingForceVector[1]], point=AcomToPoint
                ))


                if constants.SHOW_FORCES:
                    pygame.draw.line(constants.screen, "purple", calc.metresToPixels(self.pointA), 
                        calc.metresToPixels(calc.addVectors(self.pointA, opposingForceVector)), 
                    4)

            """
            # TEMP
            elif self.objectA.fixed == True:
                print("Adding Torque to A")
                self.objectA.torques.append(np.cross(AcomToPoint, calc.scalarMult(perpImpactVector, force)) / self.objectA.MoI)
            """

        if self.objectB.calcPhysics == True:

            if self.objectB.fixed == False:

                self.objectB.thrusts.append(physics.Thrust(
                    vector=[-opposingForceVector[0], -opposingForceVector[1]], point=BcomToPoint
                ))

                if constants.SHOW_FORCES:
                    pygame.draw.line(constants.screen, "purple", calc.metresToPixels(self.pointB), 
                        calc.metresToPixels(calc.addVectors(self.pointB, [-opposingForceVector[0], -opposingForceVector[1]])), 
                    4)

            else:

                # turn thrust into a torque
                # Just need to figure out how to sign the torque!

                self.objectB.parent.thrusts.append(physics.Thrust(
                    vector=[-opposingForceVector[0], -opposingForceVector[1]], point=BcomToPoint
                ))

                """

                torque = norm(opposingForceVector) * norm(BcomToPoint)

                print("Torque B", torque)

                self.objectB.parent.torques.append(torque)
                """

                if constants.SHOW_FORCES:
                    pygame.draw.line(constants.screen, "purple", calc.metresToPixels(self.pointB), 
                        calc.metresToPixels(calc.addVectors(self.pointB, [-opposingForceVector[0], -opposingForceVector[1]])), 
                    4)

            """
            # TEMP
            elif self.objectB.fixed == True:
                print("Adding Torque to B")
                self.objectB.torques.append(np.cross(BcomToPoint, calc.scalarMult(perpImpactVector, force)) / self.objectB.MoI)
            """

        # print()


    def resolveLineLine(self):

        # TEMP CODE THAT ONLY WORKS FOR ANGULAR MOMENTUM
        # assumes parents, etc.

   
        print("LINE LINE CONTACT FORCES")

        """

        print(self.objectA.id, self.objectB.id)

        torque = -2 * physics.swingTorque(self.objectA.parent)
        acc = (torque / self.objectA.parent.MoI) * constants.TIME_STEP

        print("angular V before", self.objectA.parent.angularVelocity)

        self.objectA.parent.angularVelocity -= acc

        print("angular V after", self.objectA.parent.angularVelocity)

        angleDelta = self.objectA.parent.angularVelocity * constants.TIME_STEP
        self.objectA.parent.angle -= angleDelta

        self.objectA.parent.updateCoM()

        # if moving the object root, do this here

        for object in self.objectA.parent.objects:
            object.root = self.objectA.parent.root
            object.CoM = self.objectA.parent.CoM
            object.angle -= angleDelta
            object.velocity = self.objectA.parent.velocity
            object.angularVelocity = self.objectA.parent.angularVelocity
            object.update()

        
        acc = (torque / self.objectB.parent.MoI) * constants.TIME_STEP

        print("angular V before", self.objectB.parent.angularVelocity)

        self.objectB.parent.angularVelocity -= acc

        print("angular V after", self.objectB.parent.angularVelocity)

        angleDelta = self.objectB.parent.angularVelocity * constants.TIME_STEP
        self.objectB.parent.angle -= angleDelta

        self.objectB.parent.updateCoM()

        # if moving the object root, do this here

        for object in self.objectB.parent.objects:
            object.root = self.objectB.parent.root
            object.CoM = self.objectB.parent.CoM
            object.angle -= angleDelta
            object.velocity = self.objectB.parent.velocity
            object.angularVelocity = self.objectB.parent.angularVelocity
            object.update()

        print()

        """


    """
    def resolveBallWall(self):

        # get next object velocities
        # only interested in linear velocities
        nextObjectVelocities = physics.resultOfForces(self.objectA)

        # Get component of velocity that is towards wall    
        # closingVelocity = np.dot(self.objectA.nextVelocity, self.perpImpactVector)  # scalar
        closingVelocity = np.dot(nextObjectVelocities["velocity"], self.perpImpactVector)  # scalar

        # objects are actually moving apart
        if closingVelocity > 0:
            return None

        # force to cancel closingVelocity
        opposingForce = (self.objectA.mass * -closingVelocity) / constants.TIME_STEP

        opposingForceVector = calc.scaleVector(self.perpImpactVector, opposingForce)

        opposingForce = physics.Thrust(vector=opposingForceVector, point=[0,0])
        self.objectA.thrusts.append(opposingForce)
    """