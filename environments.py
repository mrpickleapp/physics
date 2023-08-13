import objects
import constants
import input
import numpy as np
import calc
import physics

def getAvatar(screenObjects, objectCount, radius=0.1):

    avatar = objects.Ball(
        id = objectCount,
        CoM = [-1, -0.5], 
        radius = radius,
        mass = calc.circleArea(radius),
        colour = "black")

    return avatar, objectCount + 1


# standard benchmark
def loadBenchmark(screenObjects, objectCount):

    # SET VARIABLES
    constants.TICK = 60
    constants.SIM_MULTIPLIER = 50
    constants.SPEED = 0.5
    constants.GRAVITY = 9.81
    constants.FRICTION = 0.1
    constants.ELASTICITY = 0.5
    constants.DEBUG_MODE = False
    constants.DEBUG_PANEL = False
    constants.GENERAL_DAMPING = 0
    constants.COLLISION_PRUNE_PERP_F = 0.75
    constants.COLLISION_PRUNE_END_F = 1.75 

    constants.TIME_STEP = constants.setTimeStep()

    objectCount = loadWalls(screenObjects, objectCount)

    comp_1 = objects.Line(
        id = objectCount,
        mass = 0.1,
        root = [0, -0.15],
        rootToStart = [-0.2, 0],
        rootToEnd = [0.2, 0],
        colour = "olive",
        width = 0.02,
        calcPhysics = True,
        canCollide = True,
        checkCollisions = True
    )
    objectCount += 1

    comp_2 = objects.Line(
        id = objectCount,
        mass = 0.1,
        colour = "brown",
        root = [0, -0.15],
        rootToStart = [0, 0.2],
        rootToEnd = [0, -0.2],
        width = 0.02,
        calcPhysics = True,
        canCollide = True, 
        checkCollisions = True
    )
    objectCount += 1

    # system gets created last - assigns parenthood to children
    sys_1 = objects.System(
        id = objectCount,
        objects = [comp_1, comp_2],
        root = [0, -0.15],
        CoM = [0, -0.15],
        fixed = False,
        hasGravity=True
    )
    objectCount += 1

    screenObjects[sys_1.id] = sys_1
    screenObjects[comp_1.id] = comp_1
    screenObjects[comp_2.id] = comp_2

    # cross
    comp_3 = objects.Line(
        id = objectCount,
        mass = 0.1,
        colour = "red",
        root = [0.05, 0.3],
        rootToStart = [0, 0.4],
        rootToEnd = [0, -0.4],
        width = 0.02,
        calcPhysics = True,
        canCollide = True,
        checkCollisions = True
    )
    objectCount += 1

    comp_4 = objects.Line(
        id = objectCount,
        mass = 0.1,
        colour = "brown",
        root = [0.05, 0.3],
        rootToStart = [0.4, 0],
        rootToEnd = [-0.4, 0],
        width = 0.02,
        calcPhysics = True,
        canCollide = True, 
        checkCollisions = True
    )
    objectCount += 1

    # system gets created last - assigns parenthood to children
    sys_2 = objects.System(
        id = objectCount,
        objects = [comp_3, comp_4],
        root = [0.05, 0.3],
        CoM = [0.25, 0.3],
        fixed = True,
        hasGravity=True
    )
    objectCount += 1

    screenObjects[sys_2.id] = sys_2
    screenObjects[comp_3.id] = comp_3
    screenObjects[comp_4.id] = comp_4

    # random line
    line = objects.Line(
        id = objectCount,
        mass = 0.1,
        colour = "purple",
        root = [-0.8, -0.2],
        rootToStart = [-0.1, 0],
        rootToEnd = [0.3, 0],
        width = 0.02,
    )
    screenObjects[line.id] = line
    objectCount += 1

    r = 0.15
    b1 = objects.Ball(
        id = objectCount,
        CoM = [-1, 0.7], 
        radius = r,
        mass = calc.circleArea(r),
        colour = "gold")
    screenObjects[b1.id] = b1
    objectCount += 1

    r = 0.1
    b2 = objects.Ball(
        id = objectCount,
        CoM = [1, 0.6], 
        radius = r,
        mass = calc.circleArea(r),
        colour = "pink")
    screenObjects[b2.id] = b2
    objectCount += 1

    r = 0.2
    b3 = objects.Ball(
        id = objectCount,
        CoM = [0.8, -0.4], 
        radius = r,
        mass = calc.circleArea(r),
        colour = "orange")
    screenObjects[b3.id] = b3
    objectCount += 1

    r = 0.03
    b4 = objects.Ball(
        id = objectCount,
        CoM = [0.2, 0.6], 
        radius = r,
        mass = calc.circleArea(r),
        colour = "olive")
    screenObjects[b4.id] = b4
    objectCount += 1

    r = 0.06
    b5 = objects.Ball(
        id = objectCount,
        CoM = [-0.5, 0.3], 
        radius = r,
        mass = calc.circleArea(r),
        colour = "blue")
    screenObjects[b5.id] = b5
    objectCount += 1
    

def loadWalls(screenObjects, objectCount):

    # walls
    floor = objects.Wall(
        id = objectCount,
        colour = "blue",
        start = [-1.55, -0.95],
        end = [1.55, -0.95],
        width = 0.02
    )
    screenObjects[floor.id] = floor
    objectCount += 1

    ceiling = objects.Wall(
        id = objectCount,
        colour = "blue",
        start = [-1.55, 0.95],
        end = [1.55, 0.95],
        width = 0.02
    )
    screenObjects[ceiling.id] = ceiling
    objectCount += 1

    wall_l = objects.Wall(
        id = objectCount,
        colour = "blue",
        start = [-1.55, -0.95],
        end = [-1.55, 0.95],
        width = 0.02
    )
    screenObjects[wall_l.id] = wall_l
    objectCount += 1

    wall_r = objects.Wall(
        id = objectCount,
        colour = "blue",
        start = [1.55, -0.95],
        end = [1.55, 0.95],
        width = 0.02
    )
    screenObjects[wall_r.id] = wall_r
    objectCount += 1

    return objectCount


# WORKING - requires simple MoI
def clock1(screenObjects, objectCount, comparison=True, addAvatar=False):

    objectCount = loadWalls(screenObjects, objectCount)
    avatar, objectCount = getAvatar(screenObjects, objectCount)
    if addAvatar:
        screenObjects[avatar.id] = avatar

    # SET VARIABLES
    constants.TICK = 60
    constants.SIM_MULTIPLIER = 2
    constants.SPEED = 0.5  
    constants.GRAVITY = 9.81
    constants.FRICTION = 0.01
    constants.ELASTICITY = 0.2
    constants.DEBUG_MODE = False
    constants.DEBUG_PANEL = False
    constants.GENERAL_DAMPING = 0.25
    constants.COLLISION_PRUNE_PERP_F = 0.75
    constants.COLLISION_PRUNE_END_F = 1.75 

    constants.TIME_STEP = constants.setTimeStep()

    # escapement
    esc_root = [0, 0.6]

    top_bar_l = objects.Line(
        id = objectCount,
        mass = 0.1,
        colour = "gold",
        root = esc_root,
        rootToStart = [0, 0],
        rootToEnd = [-0.3, -0.1],
        width = 0.02,
        calcPhysics = False,
        canCollide = False,
        checkCollisions = False
    )
    objectCount += 1

    top_bar_r = objects.Line(
        id = objectCount,
        mass = 0.1,
        colour = "gold",
        root = esc_root,
        rootToStart = [0, 0],
        rootToEnd = [0.3, -0.1],
        width = 0.02,
        calcPhysics = False,
        canCollide = False,
        checkCollisions = False
    )
    objectCount += 1

    hr = objects.Line(
        id = objectCount,
        mass = 0.1,
        colour = "gold",
        root = esc_root,
        rootToStart = [0.25, -0.09],
        rootToEnd = [0.21, -0.21],
        width = 0.02,
        calcPhysics = True,
        canCollide = True,
        checkCollisions = True
    )
    objectCount += 1

    lr = objects.Line(
        id = objectCount,
        mass = 0.1,
        colour = "gold",
        root = esc_root,
        rootToStart = [-0.3, -0.1],
        rootToEnd = [-0.1, -0.12],
        width = 0.02,
        calcPhysics = True,
        canCollide = True,
        checkCollisions = True
    )
    objectCount += 1

    vert = objects.Line(
        id = objectCount,
        mass = 0.1,
        colour = "gold",
        root = esc_root,
        rootToStart = [0, 0],
        rootToEnd = [0, -1.5],
        width = 0.02,
        calcPhysics = False,
        canCollide = False,
        checkCollisions = False
    )
    objectCount += 1

    bob = objects.Line(
        id = objectCount,
        mass = 1,
        colour = "gold",
        root = esc_root,
        rootToStart = [-0.1, -1.4],
        rootToEnd = [0.1, -1.4],
        width = 0.05,
        calcPhysics = False,
        canCollide = True,
        checkCollisions = False
    )
    objectCount += 1

    # system gets created last - assigns parenthood to children
    escapement = objects.System(
        id = objectCount,
        objects = [top_bar_l, lr, top_bar_r, hr, vert, bob],
        root = esc_root,
        CoM = [0, -0.8],
        fixed = True,
        hasGravity=True,
        sumMoI=True
    )
    objectCount += 1

    screenObjects[escapement.id] = escapement
    screenObjects[top_bar_l.id] = top_bar_l
    screenObjects[lr.id] = lr
    screenObjects[top_bar_r.id] = top_bar_r
    screenObjects[hr.id] = hr
    screenObjects[vert.id] = vert
    screenObjects[bob.id] = bob

    startingV = -2
    escapement.angularVelocity = startingV

    # GEAR

    gearCentre = [0, 0.2]
    gearRadius = 0.16
    toothCount = 13

    gear_wheel = objects.Ball(
        id = objectCount,
        CoM = [0, 0.2],
        radius = gearRadius,
        mass = 0,
        colour = "orange",
        hasGravity = False,
        canCollide = False,
        checkCollisions=False,
        calcPhysics=False,
        fixed=True
    )
    objectCount += 1
    screenObjects[gear_wheel.id] = gear_wheel

    teeth = []

    for tooth in range(toothCount):

        angle = (np.radians(360) / toothCount) * tooth

        rootToStart = [0.15, 0]
        rootToStart = calc.rotateVector(rootToStart, angle)

        rootToEnd = [0.15, -0.25]
        rootToEnd = calc.rotateVector(rootToEnd, angle)

        tooth = objects.Line(
            id = objectCount,
            mass = 0.01,
            colour = "darkorange",
            root = gearCentre,
            rootToStart=rootToStart,
            rootToEnd=rootToEnd,
            width = 0.02,
            calcPhysics=True,
            canCollide=True,
            checkCollisions=False
        )
        objectCount += 1

        teeth.append(tooth)

    gearSystem = objects.System(
        id = objectCount,
        objects = teeth,
        root = gearCentre,
        CoM = gearCentre,
        fixed = True,
        hasGravity = True,
        sumMoI=True
    )
    objectCount += 1

    screenObjects[gearSystem.id] = gearSystem

    for tooth in teeth:
        screenObjects[tooth.id] = tooth

    gearTorque = 0.05

    def clock1Torque():
        physics.applyTorque(gearSystem, gearTorque)

    constants.simIterationFunctions.append(clock1Torque)

    #----------------------------------------------

    if not(comparison):
        return avatar, objectCount

    esc_rootm = [0.8, 0.6]

    top_bar_lm = objects.Line(
        id = objectCount,
        mass = 0.1,
        colour = "grey",
        root = esc_rootm,
        rootToStart = [0, 0],
        rootToEnd = [-0.3, -0.1],
        width = 0.02,
        calcPhysics = False,
        canCollide = False,
        checkCollisions = False
    )
    objectCount += 1

    top_bar_rm = objects.Line(
        id = objectCount,
        mass = 0.1,
        colour = "grey",
        root = esc_rootm,
        rootToStart = [0, 0],
        rootToEnd = [0.3, -0.1],
        width = 0.02,
        calcPhysics = False,
        canCollide = False,
        checkCollisions = False
    )
    objectCount += 1

    hrm = objects.Line(
        id = objectCount,
        mass = 0.1,
        colour = "grey",
        root = esc_rootm,
        rootToStart = [0.25, -0.09],
        rootToEnd = [0.21, -0.21],
        width = 0.02,
        calcPhysics = False,
        canCollide = False,
        checkCollisions = False
    )
    objectCount += 1

    lrm = objects.Line(
        id = objectCount,
        mass = 0.1,
        colour = "grey",
        root = esc_rootm,
        rootToStart = [-0.3, -0.1],
        rootToEnd = [-0.12, -0.12],
        width = 0.02,
        calcPhysics = False,
        canCollide = False,
        checkCollisions = False
    )
    objectCount += 1

    vertm = objects.Line(
        id = objectCount,
        mass = 0.1,
        colour = "grey",
        root = esc_rootm,
        rootToStart = [0, 0],
        rootToEnd = [0, -1.5],
        width = 0.02,
        calcPhysics = False,
        canCollide = False,
        checkCollisions = False
    )
    objectCount += 1

    bobm = objects.Line(
        id = objectCount,
        mass = 10,
        colour = "grey",
        root = esc_rootm,
        rootToStart = [-0.1, -1.4],
        rootToEnd = [0.1, -1.4],
        width = 0.05,
        calcPhysics = False,
        canCollide = False,
        checkCollisions = False
    )
    objectCount += 1

    # system gets created last - assigns parenthood to children
    escapementm = objects.System(
        id = objectCount,
        objects = [top_bar_lm, lrm, top_bar_rm, hrm, vertm, bobm],
        root = esc_rootm,
        CoM = [0.8, -0.8],
        fixed = True,
        hasGravity=True
    )
    objectCount += 1

    screenObjects[escapementm.id] = escapementm
    screenObjects[top_bar_lm.id] = top_bar_lm
    screenObjects[lrm.id] = lrm
    screenObjects[top_bar_rm.id] = top_bar_rm
    screenObjects[hrm.id] = hrm
    screenObjects[vertm.id] = vertm
    screenObjects[bobm.id] = bobm

    escapementm.angularVelocity = startingV

    return avatar, objectCount


# MoI calculated more accurately
    # bob is main contribution to MoI
# this should work when contact forces are being calced properly
# works v nicely with elasticity set to 1
# otherwise, lines start to clip through one another
# test again with resting forces
def clock2(screenObjects, objectCount, comparison=True, addAvatar=False):

    objectCount = loadWalls(screenObjects, objectCount)
    avatar, objectCount = getAvatar(screenObjects, objectCount)
    if addAvatar:
        screenObjects[avatar.id] = avatar

    # SET VARIABLES
    constants.TICK = 60
    constants.SIM_MULTIPLIER = 10
    constants.SPEED = 1  
    constants.GRAVITY = 9.81
    constants.FRICTION = 0.01
    constants.ELASTICITY = 0.25
    constants.DEBUG_MODE = False
    constants.DEBUG_PANEL = False
    constants.GENERAL_DAMPING = 0.01
    constants.COLLISION_PRUNE_PERP_F = 0.75
    constants.COLLISION_PRUNE_END_F = 1.75 

    constants.TIME_STEP = constants.setTimeStep()

    constants.SHOW_FORCES = False

    # escapement
    esc_root = [0, 0.6]

    top_bar_l = objects.Line(
        id = objectCount,
        mass = 0.01,
        colour = "gold",
        root = esc_root,
        rootToStart = [0, 0],
        rootToEnd = [-0.3, -0.1],
        width = 0.02,
        calcPhysics = False,
        canCollide = True,
        checkCollisions = True
    )
    objectCount += 1

    top_bar_r = objects.Line(
        id = objectCount,
        mass = 0.01,
        colour = "gold",
        root = esc_root,
        rootToStart = [0, 0],
        rootToEnd = [0.3, -0.1],
        width = 0.02,
        calcPhysics = False,
        canCollide = True,
        checkCollisions = True
    )
    objectCount += 1

    hr = objects.Line(
        id = objectCount,
        mass = 0.01,
        colour = "gold",
        root = esc_root,
        rootToStart = [0.25, -0.09],
        rootToEnd = [0.21, -0.21],
        width = 0.02,
        calcPhysics = True,
        canCollide = True,
        checkCollisions = True
    )
    objectCount += 1

    lr = objects.Line(
        id = objectCount,
        mass = 0.01,
        colour = "gold",
        root = esc_root,
        rootToStart = [-0.3, -0.1],
        rootToEnd = [-0.1, -0.13],
        width = 0.02,
        calcPhysics = True,
        canCollide = True,
        checkCollisions = True
    )
    objectCount += 1

    vert = objects.Line(
        id = objectCount,
        mass = 0.1,
        colour = "gold",
        root = esc_root,
        rootToStart = [0, 0],
        rootToEnd = [0, -1.5],
        width = 0.02,
        calcPhysics = False,
        canCollide = False,
        checkCollisions = False
    )
    objectCount += 1

    bob = objects.Line(
        id = objectCount,
        mass = 1,
        colour = "gold",
        root = esc_root,
        rootToStart = [-0.1, -1.4],
        rootToEnd = [0.1, -1.4],
        width = 0.05,
        calcPhysics = True,
        canCollide = True,
        checkCollisions = True
    )
    objectCount += 1

    # system gets created last - assigns parenthood to children
    escapement = objects.System(
        id = objectCount,
        objects = [top_bar_l, lr, top_bar_r, hr, vert, bob],
        root = esc_root,
        CoM = [0, -0.8],
        fixed = True,
        hasGravity=True,
        sumMoI=False,
        angle=0.08
    )
    objectCount += 1

    screenObjects[escapement.id] = escapement
    screenObjects[top_bar_l.id] = top_bar_l
    screenObjects[lr.id] = lr
    screenObjects[top_bar_r.id] = top_bar_r
    screenObjects[hr.id] = hr
    screenObjects[vert.id] = vert
    screenObjects[bob.id] = bob

    startingV = 0
    escapement.angularVelocity = startingV

    # GEAR

    gearCentre = [0, 0.2]
    gearRadius = 0.16
    toothCount = 13

    gear_wheel = objects.Ball(
        id = objectCount,
        CoM = [0, 0.2],
        radius = gearRadius,
        mass = 0,
        colour = "orange",
        hasGravity = False,
        canCollide = False,
        checkCollisions=False,
        calcPhysics=False,
        fixed=True
    )
    objectCount += 1
    screenObjects[gear_wheel.id] = gear_wheel

    teeth = []

    for tooth in range(toothCount):

        angle = (np.radians(360) / toothCount) * tooth

        rootToStart = [0.15, 0]
        rootToStart = calc.rotateVector(rootToStart, angle)

        rootToEnd = [0.15, -0.232]
        rootToEnd = calc.rotateVector(rootToEnd, angle)

        tooth = objects.Line(
            id = objectCount,
            mass = 0.005,
            colour = "darkorange",
            root = gearCentre,
            rootToStart=rootToStart,
            rootToEnd=rootToEnd,
            width = 0.02,
            calcPhysics=True,
            canCollide=True,
            checkCollisions=False
        )
        objectCount += 1

        teeth.append(tooth)

    gearSystem = objects.System(
        id = objectCount,
        objects = teeth,
        root = gearCentre,
        CoM = gearCentre,
        fixed = True,
        hasGravity = True,
        calcPhysics=True,
        sumMoI=False
    )
    objectCount += 1

    screenObjects[gearSystem.id] = gearSystem

    for tooth in teeth:
        screenObjects[tooth.id] = tooth

    gearTorque = 0.05

    def clock1Torque():
        physics.applyTorque(gearSystem, gearTorque)

    constants.simIterationFunctions.append(clock1Torque)

    #----------------------------------------------

    if not(comparison):
        return avatar, objectCount

    esc_rootm = [0.8, 0.6]

    top_bar_lm = objects.Line(
        id = objectCount,
        mass = 0.1,
        colour = "grey",
        root = esc_rootm,
        rootToStart = [0, 0],
        rootToEnd = [-0.3, -0.1],
        width = 0.02,
        calcPhysics = False,
        canCollide = False,
        checkCollisions = False
    )
    objectCount += 1

    top_bar_rm = objects.Line(
        id = objectCount,
        mass = 0.1,
        colour = "grey",
        root = esc_rootm,
        rootToStart = [0, 0],
        rootToEnd = [0.3, -0.1],
        width = 0.02,
        calcPhysics = False,
        canCollide = False,
        checkCollisions = False
    )
    objectCount += 1

    hrm = objects.Line(
        id = objectCount,
        mass = 0.1,
        colour = "grey",
        root = esc_rootm,
        rootToStart = [0.25, -0.09],
        rootToEnd = [0.21, -0.21],
        width = 0.02,
        calcPhysics = False,
        canCollide = False,
        checkCollisions = False
    )
    objectCount += 1

    lrm = objects.Line(
        id = objectCount,
        mass = 0.1,
        colour = "grey",
        root = esc_rootm,
        rootToStart = [-0.3, -0.1],
        rootToEnd = [-0.12, -0.12],
        width = 0.02,
        calcPhysics = False,
        canCollide = False,
        checkCollisions = False
    )
    objectCount += 1

    vertm = objects.Line(
        id = objectCount,
        mass = 0.1,
        colour = "grey",
        root = esc_rootm,
        rootToStart = [0, 0],
        rootToEnd = [0, -1.5],
        width = 0.02,
        calcPhysics = False,
        canCollide = False,
        checkCollisions = False
    )
    objectCount += 1

    bobm = objects.Line(
        id = objectCount,
        mass = 10,
        colour = "grey",
        root = esc_rootm,
        rootToStart = [-0.1, -1.4],
        rootToEnd = [0.1, -1.4],
        width = 0.05,
        calcPhysics = False,
        canCollide = False,
        checkCollisions = False
    )
    objectCount += 1

    # system gets created last - assigns parenthood to children
    escapementm = objects.System(
        id = objectCount,
        objects = [top_bar_lm, lrm, top_bar_rm, hrm, vertm, bobm],
        root = esc_rootm,
        CoM = [0.8, -0.8],
        fixed = True,
        hasGravity=True
    )
    objectCount += 1

    screenObjects[escapementm.id] = escapementm
    screenObjects[top_bar_lm.id] = top_bar_lm
    screenObjects[lrm.id] = lrm
    screenObjects[top_bar_rm.id] = top_bar_rm
    screenObjects[hrm.id] = hrm
    screenObjects[vertm.id] = vertm
    screenObjects[bobm.id] = bobm

    escapementm.angularVelocity = startingV

    return avatar, objectCount


def everything(screenObjects, objectCount, comparison=False, addAvatar=False):

    objectCount = loadWalls(screenObjects, objectCount)
    avatar, objectCount = getAvatar(screenObjects, objectCount)
    if addAvatar:
        screenObjects[avatar.id] = avatar

    # SET VARIABLES
    constants.TICK = 60
    constants.SIM_MULTIPLIER = 10
    constants.SPEED = 0.75  
    constants.GRAVITY = 9.81
    constants.FRICTION = 0
    constants.ELASTICITY = 1
    constants.DEBUG_MODE = False
    constants.DEBUG_PANEL = False
    constants.GENERAL_DAMPING = 0.0
    constants.COLLISION_PRUNE_PERP_F = 0.75
    constants.COLLISION_PRUNE_END_F = 1.75 

    constants.TIME_STEP = constants.setTimeStep()

    constants.SHOW_FORCES = False

    # escapement
    esc_root = [0.8, 0.6]

    top_bar_l = objects.Line(
        id = objectCount,
        mass = 0.01,
        colour = "gold",
        root = esc_root,
        rootToStart = [0, 0],
        rootToEnd = [-0.3, -0.1],
        width = 0.02,
        calcPhysics = False,
        canCollide = True,
        checkCollisions = True
    )
    objectCount += 1

    top_bar_r = objects.Line(
        id = objectCount,
        mass = 0.01,
        colour = "gold",
        root = esc_root,
        rootToStart = [0, 0],
        rootToEnd = [0.3, -0.1],
        width = 0.02,
        calcPhysics = False,
        canCollide = True,
        checkCollisions = True
    )
    objectCount += 1

    hr = objects.Line(
        id = objectCount,
        mass = 0.01,
        colour = "gold",
        root = esc_root,
        rootToStart = [0.25, -0.09],
        rootToEnd = [0.21, -0.21],
        width = 0.02,
        calcPhysics = True,
        canCollide = True,
        checkCollisions = True
    )
    objectCount += 1

    lr = objects.Line(
        id = objectCount,
        mass = 0.01,
        colour = "gold",
        root = esc_root,
        rootToStart = [-0.3, -0.1],
        rootToEnd = [-0.1, -0.13],
        width = 0.02,
        calcPhysics = True,
        canCollide = True,
        checkCollisions = True
    )
    objectCount += 1

    vert = objects.Line(
        id = objectCount,
        mass = 0.1,
        colour = "gold",
        root = esc_root,
        rootToStart = [0, 0],
        rootToEnd = [0, -1.5],
        width = 0.02,
        calcPhysics = False,
        canCollide = False,
        checkCollisions = False
    )
    objectCount += 1

    bob = objects.Line(
        id = objectCount,
        mass = 1,
        colour = "gold",
        root = esc_root,
        rootToStart = [-0.1, -1.4],
        rootToEnd = [0.1, -1.4],
        width = 0.05,
        calcPhysics = True,
        canCollide = True,
        checkCollisions = True
    )
    objectCount += 1

    # system gets created last - assigns parenthood to children
    escapement = objects.System(
        id = objectCount,
        objects = [top_bar_l, lr, top_bar_r, hr, vert, bob],
        root = esc_root,
        CoM = [0.8, -0.8],
        fixed = True,
        hasGravity=True,
        sumMoI=False
    )
    objectCount += 1

    screenObjects[escapement.id] = escapement
    screenObjects[top_bar_l.id] = top_bar_l
    screenObjects[lr.id] = lr
    screenObjects[top_bar_r.id] = top_bar_r
    screenObjects[hr.id] = hr
    screenObjects[vert.id] = vert
    screenObjects[bob.id] = bob

    startingV = -0.2
    escapement.angularVelocity = startingV

    # GEAR

    gearCentre = [0.8, 0.2]
    gearRadius = 0.16
    toothCount = 13

    gear_wheel = objects.Ball(
        id = objectCount,
        CoM = [0.8, 0.2],
        radius = gearRadius,
        mass = 0,
        colour = "orange",
        hasGravity = False,
        canCollide = False,
        checkCollisions=False,
        calcPhysics=False,
        fixed=True
    )
    objectCount += 1
    screenObjects[gear_wheel.id] = gear_wheel

    teeth = []

    for tooth in range(toothCount):

        angle = (np.radians(360) / toothCount) * tooth

        rootToStart = [0.15, 0]
        rootToStart = calc.rotateVector(rootToStart, angle)

        rootToEnd = [0.15, -0.232]
        rootToEnd = calc.rotateVector(rootToEnd, angle)

        tooth = objects.Line(
            id = objectCount,
            mass = 0.005,
            colour = "darkorange",
            root = gearCentre,
            rootToStart=rootToStart,
            rootToEnd=rootToEnd,
            width = 0.02,
            calcPhysics=True,
            canCollide=True,
            checkCollisions=False
        )
        objectCount += 1

        teeth.append(tooth)

    gearSystem = objects.System(
        id = objectCount,
        objects = teeth,
        root = gearCentre,
        CoM = gearCentre,
        fixed = True,
        hasGravity = True,
        calcPhysics=True,
        sumMoI=False
    )
    objectCount += 1

    screenObjects[gearSystem.id] = gearSystem

    for tooth in teeth:
        screenObjects[tooth.id] = tooth

    gearTorque = 0.05

    def clock1Torque():
        physics.applyTorque(gearSystem, gearTorque)

    constants.simIterationFunctions.append(clock1Torque)

    

    return avatar, objectCount



def restingForces(screenObjects, objectCount):

    # SET VARIABLES
    constants.TICK = 60
    constants.SIM_MULTIPLIER = 10
    constants.SPEED = 0.5
    constants.GRAVITY = 9.81
    constants.FRICTION = 0.0
    constants.ELASTICITY = 0
    constants.DEBUG_MODE = False
    constants.DEBUG_PANEL = False
    constants.GENERAL_DAMPING = 0
    constants.COLLISION_PRUNE_PERP_F = 0.75
    constants.COLLISION_PRUNE_END_F = 1.75 

    constants.SHOW_FORCES = False

    constants.TIME_STEP = constants.setTimeStep()

    objectCount = loadWalls(screenObjects, objectCount)

    comp_1 = objects.Line(
        id = objectCount,
        mass = 0.1,
        root = [0, 0],
        rootToStart = [-0.4, 0],
        rootToEnd = [0.4, 0],
        colour = "olive",
        width = 0.02,
        calcPhysics = True,
        canCollide = True,
        checkCollisions = True
    )
    objectCount += 1

    comp_2 = objects.Line(
        id = objectCount,
        mass = 0.1,
        colour = "brown",
        root = [0, 0],
        rootToStart = [0, 0.4],
        rootToEnd = [0, -0.4],
        width = 0.02,
        calcPhysics = True,
        canCollide = True, 
        checkCollisions = True
    )
    objectCount += 1

    # system gets created last - assigns parenthood to children
    sys_1 = objects.System(
        id = objectCount,
        objects = [comp_1, comp_2],
        root = [0, 0],
        CoM = [0.2, 0],
        fixed = True,
        hasGravity=True
    )
    objectCount += 1

    screenObjects[sys_1.id] = sys_1
    screenObjects[comp_1.id] = comp_1
    screenObjects[comp_2.id] = comp_2

    comp_3 = objects.Line(
        id = objectCount,
        mass = 10,
        root = [0.6, -0.1],
        rootToStart = [-0.4, 0],
        rootToEnd = [0.4, 0],
        colour = "gold",
        width = 0.02,
        calcPhysics = True,
        canCollide = True,
        checkCollisions = True
    )
    objectCount += 1

    comp_4 = objects.Line(
        id = objectCount,
        mass = 10,
        colour = "orange",
        root = [0.6, -0.1],
        rootToStart = [0, 0.4],
        rootToEnd = [0, -0.4],
        width = 0.02,
        calcPhysics = True,
        canCollide = True, 
        checkCollisions = True
    )
    objectCount += 1

    # system gets created last - assigns parenthood to children
    sys_2 = objects.System(
        id = objectCount,
        objects = [comp_3, comp_4],
        root = [0.6, -0.1],
        CoM = [0.6, -0.1],
        fixed = True,
        hasGravity=True
    )
    objectCount += 1

    screenObjects[sys_2.id] = sys_2
    screenObjects[comp_3.id] = comp_3
    screenObjects[comp_4.id] = comp_4

    r = 0.2
    avatar = objects.Ball(
        id = objectCount,
        CoM = [-0.5, -0.6], 
        radius = r,
        mass = calc.circleArea(r),
        colour="black"
    )
    objectCount += 1
    screenObjects[avatar.id] = avatar

    r = 0.14

    ball = objects.Ball(
        id = objectCount,
        CoM = [-0.9, -0.6], 
        radius = r,
        mass = calc.circleArea(r),
        colour="olive"
    )
    objectCount += 1
    screenObjects[ball.id] = ball

    ball2 = objects.Ball(
        id = objectCount,
        CoM = [-0.9, -0.3], 
        radius = r,
        mass = calc.circleArea(r),
        colour="tan"
    )
    objectCount += 1
    # screenObjects[ball2.id] = ball2

    ball3 = objects.Ball(
        id = objectCount,
        CoM = [-0.9, 0.1], 
        radius = r,
        mass = calc.circleArea(r),
        colour="beige"
    )
    objectCount += 1
    # screenObjects[ball3.id] = ball3

    # random line
    line = objects.Line(
        id = objectCount,
        mass = 0.1,
        colour = "olive",
        root = [-0.86, -0.57],
        rootToStart = [-0.1, 0],
        rootToEnd = [0.3, 0],
        width = 0.02,
        fixed=False
    )
    # screenObjects[line.id] = line
    objectCount += 1

    # random line
    line2 = objects.Line(
        id = objectCount,
        mass = 0.1,
        colour = "brown",
        root = [-0.3, -0.4],
        rootToStart = [-0.2, -0.1],
        rootToEnd = [0.2, 0.1],
        width = 0.02,
        fixed=False
    )
    # screenObjects[line2.id] = line2
    objectCount += 1

    # random line
    line3 = objects.Line(
        id = objectCount,
        mass = 0.1,
        colour = "navy",
        root = [0.99, -0.7],
        rootToStart = [-0.2, -0.1],
        rootToEnd = [0.2, 0.1],
        width = 0.02,
        fixed=False
    )
    # screenObjects[line3.id] = line3
    objectCount += 1
    

    return avatar, objectCount


def demo1(screenObjects, objectCount):

    # SET VARIABLES
    constants.TICK = 60
    constants.SIM_MULTIPLIER = 5
    constants.SPEED = 0.25
    constants.GRAVITY = 9.81
    constants.FRICTION = 0.0
    constants.ELASTICITY = 0
    constants.DEBUG_MODE = False
    constants.DEBUG_PANEL = True
    constants.GENERAL_DAMPING = 0
    constants.COLLISION_PRUNE_PERP_F = 0.75
    constants.COLLISION_PRUNE_END_F = 1.75 

    constants.SHOW_FORCES = False

    constants.TIME_STEP = constants.setTimeStep()

    
    r = 0.15
    avatar = objects.Ball(
        id = objectCount,
        CoM = [0, 0.7], 
        radius = r,
        mass = calc.circleArea(r),
        colour="black"
    )
    objectCount += 1
    screenObjects[avatar.id] = avatar

    return avatar, objectCount

def demo2(screenObjects, objectCount):

    # SET VARIABLES
    constants.TICK = 60
    constants.SIM_MULTIPLIER = 10
    constants.SPEED = 0.6
    constants.GRAVITY = 9.81
    constants.FRICTION = 0.0
    constants.ELASTICITY = 1
    constants.DEBUG_MODE = False
    constants.DEBUG_PANEL = True
    constants.GENERAL_DAMPING = 0
    constants.COLLISION_PRUNE_PERP_F = 0.75
    constants.COLLISION_PRUNE_END_F = 1.75 

    constants.SHOW_FORCES = False

    constants.TIME_STEP = constants.setTimeStep()

    objectCount = loadWalls(screenObjects, objectCount)

    r = 0.2
    avatar = objects.Ball(
        id = objectCount,
        CoM = [0, 0.8], 
        radius = r,
        mass = calc.circleArea(r),
        colour="black"
    )
    objectCount += 1
    screenObjects[avatar.id] = avatar

    line = objects.Wall(
        id = objectCount,
        colour = "blue",
        start = [0.1, 0],
        end = [0.5, -0.4],
        width = 0.02
    )
    screenObjects[line.id] = line
    objectCount += 1

    line2 = objects.Wall(
        id = objectCount,
        colour = "blue",
        start = [-0.7, 0],
        end = [-0.2, 0.4],
        width = 0.02
    )
    screenObjects[line2.id] = line2
    objectCount += 1

    return avatar, objectCount

def demo3(screenObjects, objectCount):

    # SET VARIABLES
    constants.TICK = 60
    constants.SIM_MULTIPLIER = 10
    constants.SPEED = 0.4
    constants.GRAVITY = 9.81
    constants.FRICTION = 0.0
    constants.ELASTICITY = 1
    constants.DEBUG_MODE = False
    constants.DEBUG_PANEL = True
    constants.GENERAL_DAMPING = 0
    constants.COLLISION_PRUNE_PERP_F = 0.75
    constants.COLLISION_PRUNE_END_F = 1.75 

    constants.SHOW_FORCES = False

    constants.TIME_STEP = constants.setTimeStep()

    objectCount = loadWalls(screenObjects, objectCount)

    r = 0.15
    avatar = objects.Ball(
        id = objectCount,
        CoM = [0, 0.7], 
        radius = r,
        mass = calc.circleArea(r),
        colour="black"
    )
    objectCount += 1
    screenObjects[avatar.id] = avatar

    line = objects.Wall(
        id = objectCount,
        colour = "blue",
        start = [0.1, 0],
        end = [0.5, -0.4],
        width = 0.02
    )
    screenObjects[line.id] = line
    objectCount += 1

    line2 = objects.Wall(
        id = objectCount,
        colour = "blue",
        start = [-0.7, 0],
        end = [-0.2, 0.4],
        width = 0.02
    )
    screenObjects[line2.id] = line2
    objectCount += 1

    r = 0.2
    ball = objects.Ball(
        id = objectCount,
        CoM = [-0.3, -0.6], 
        radius = r,
        mass = calc.circleArea(r),
        colour="purple"
    )
    objectCount += 1
    screenObjects[ball.id] = ball

    r = 0.1
    ball2 = objects.Ball(
        id = objectCount,
        CoM = [0.3, 0.6], 
        radius = r,
        mass = calc.circleArea(r),
        colour="orange"
    )
    objectCount += 1
    screenObjects[ball2.id] = ball2

    r = 0.05
    ball3 = objects.Ball(
        id = objectCount,
        CoM = [-0.7, 0.6], 
        radius = r,
        mass = calc.circleArea(r),
        colour="red"
    )
    objectCount += 1
    screenObjects[ball3.id] = ball3

    r = 0.03
    ball3 = objects.Ball(
        id = objectCount,
        CoM = [0.5, -0.8], 
        radius = r,
        mass = calc.circleArea(r),
        colour="gold"
    )
    objectCount += 1
    screenObjects[ball3.id] = ball3

    return avatar, objectCount


def demo5(screenObjects, objectCount):

    # SET VARIABLES
    constants.TICK = 60
    constants.SIM_MULTIPLIER = 10
    constants.SPEED = 0.25
    constants.GRAVITY = 9.81
    constants.FRICTION = 0.0
    constants.ELASTICITY = 1
    constants.DEBUG_MODE = False
    constants.DEBUG_PANEL = False
    constants.GENERAL_DAMPING = 0
    constants.COLLISION_PRUNE_PERP_F = 0.75
    constants.COLLISION_PRUNE_END_F = 1.75 

    constants.SHOW_FORCES = False

    constants.TIME_STEP = constants.setTimeStep()

    objectCount = loadWalls(screenObjects, objectCount)

    r = 0.15
    avatar = objects.Ball(
        id = objectCount,
        CoM = [0, 0.7], 
        radius = r,
        mass = calc.circleArea(r),
        colour="black"
    )
    objectCount += 1
    screenObjects[avatar.id] = avatar

    r = 0.2
    ball = objects.Ball(
        id = objectCount,
        CoM = [-0.3, -0.6], 
        radius = r,
        mass = calc.circleArea(r),
        colour="purple"
    )
    objectCount += 1
    screenObjects[ball.id] = ball

    r = 0.1
    ball2 = objects.Ball(
        id = objectCount,
        CoM = [0.3, 0.6], 
        radius = r,
        mass = calc.circleArea(r),
        colour="orange"
    )
    objectCount += 1
    screenObjects[ball2.id] = ball2

    r = 0.1
    ball3 = objects.Ball(
        id = objectCount,
        CoM = [-0.7, 0.6], 
        radius = r,
        mass = calc.circleArea(r),
        colour="red"
    )
    objectCount += 1

    r = 0.2
    ball4 = objects.Ball(
        id = objectCount,
        CoM = [-0.9, 0.4], 
        radius = r,
        mass = calc.circleArea(r),
        colour="gold"
    )
    objectCount += 1

    spring = objects.Spring(
        id = objectCount,
        colour="grey",
        objectA=ball3,
        objectB = ball4,
        strength=constants.SPRING_FORCE
    )
    objectCount += 1
    screenObjects[ball4.id] = ball4
    screenObjects[ball3.id] = ball3
    # screenObjects[spring.id] = spring
    

    # random line
    line = objects.Line(
        id = objectCount,
        mass = 0.1,
        colour = "olive",
        root = [0.86, -0.57],
        rootToStart = [-0.1, 0],
        rootToEnd = [0.3, 0],
        width = 0.02,
        fixed=False
    )
    screenObjects[line.id] = line
    objectCount += 1

    # random line
    line2 = objects.Line(
        id = objectCount,
        mass = 0.1,
        colour = "brown",
        root = [0.6, -0.4],
        rootToStart = [-0.2, -0.1],
        rootToEnd = [0.2, 0.1],
        width = 0.02,
        fixed=False
    )
    screenObjects[line2.id] = line2
    objectCount += 1

    spring2 = objects.Spring(
        id = objectCount,
        colour="grey",
        objectA=line,
        objectB = line2,
        strength=constants.SPRING_FORCE
    )
    # screenObjects[spring2.id] = spring2

    line3 = objects.Line(
        id = objectCount,
        mass = 0.1,
        colour = "olive",
        root = [0.6, 0.57],
        rootToStart = [-0.2, 0],
        rootToEnd = [0.5, 0],
        width = 0.02,
        fixed=False
    )
    screenObjects[line3.id] = line3
    objectCount += 1

    # random line
    line4 = objects.Line(
        id = objectCount,
        mass = 0.1,
        colour = "brown",
        root = [0.2, -0.5],
        rootToStart = [-0.2, -0.1],
        rootToEnd = [0.2, 0.1],
        width = 0.02,
        fixed=False
    )
    screenObjects[line4.id] = line4
    objectCount += 1

    return avatar, objectCount

