import sys
import pygame
import objects
import constants
import collision
import environments
import input
import calc
from numpy import dot
import numpy as np
import physics
import math
import contacts


def main(**kwargs):

    print()
    for k, v in kwargs.items():
        print('Arg: {} = {}'.format(k, v))
    print()

    # initialize the pygame module
    pygame.init()

    # load and set the logo
    logo = pygame.image.load("logo_32x32.png")
    pygame.display.set_icon(logo)
    pygame.display.set_caption("minimal program")
     
    # create a surface on screen
    constants.screen = pygame.display.set_mode((constants.screenPixels[0], constants.screenPixels[1]))
     
    # define a variable to control the main loop
    running = True

    # clock
    clock = pygame.time.Clock() 

    objectCount = 0
    screenObjects = {}    

    radius = 0.1
    avatar = objects.Ball(
        id = objectCount,
        CoM = [-1, -0.5], 
        radius = radius,
        mass = calc.circleArea(radius),
        colour = "black")
    # screenObjects[avatar.id] = avatar
    # objectCount += 1
    inputThrust = objects.Thrust([0, 0], [0, 0])


    env = kwargs.get('env')
    match env:
        case 'bench':
            environments.loadBenchmark(screenObjects, objectCount)
        case 'benchmark':
            environments.loadBenchmark(screenObjects, objectCount)
        case 'clock':
            avatar, objectCount = environments.clock1(screenObjects, objectCount, comparison=False)
        case 'clock1':
            avatar, objectCount = environments.clock1(screenObjects, objectCount, comparison=True, addAvatar=True)  
        case 'clock2':
            avatar, objectCount = environments.clock2(screenObjects, objectCount, comparison=False, addAvatar=False)  
        case 'demo':
            avatar, objectCount = environments.demo1(screenObjects, objectCount)
        case 'restingForces':
            avatar, objectCount = environments.restingForces(screenObjects, objectCount)
        case 'demo1':       # gravity
            avatar, objectCount = environments.demo1(screenObjects, objectCount)
        case 'demo2':       # ball collision with wall
            avatar, objectCount = environments.demo2(screenObjects, objectCount)
        case 'demo3':       # ball collision with wall
            avatar, objectCount = environments.demo3(screenObjects, objectCount)
        case 'demo5':       # ball collision with wall
            avatar, objectCount = environments.demo5(screenObjects, objectCount)
        case 'all':       
            avatar, objectCount = environments.everything(screenObjects, objectCount)


    rec = kwargs.get('record')

    # Create a screen recorder object
    if rec:
        constants.outputFileName = kwargs.get('fname')
        import record
        recorder = record.ScreenRecorder(constants.screenPixels[0], constants.screenPixels[1], constants.TICK)

    # ----------------------------------------------------------------

    # currently SPRINGS have to be added BEFORE connected objects to calc phys properly

    fontSize = 16
    font = pygame.font.Font('freesansbold.ttf', fontSize)

    frameRates = []
    # Report avg fps after 10s
    reported = False
    reportTime = 5
     
    # main loop
    while running:

        clock.tick(constants.TICK)

        # event handling, gets all event from the event queue
        for event in pygame.event.get():

            # KEYBOARD INPUT - AVATAR
            if event.type == pygame.KEYDOWN:

                if event.scancode == 82:
                    print("UP")
                    inputThrust.vector[1] += constants.INPUT_FORCE

                elif event.scancode == 81:
                    print("DOWN")
                    inputThrust.vector[1] -= constants.INPUT_FORCE

                elif event.scancode == 80:
                    print("LEFT")
                    inputThrust.vector[0] -= constants.INPUT_FORCE

                elif event.scancode == 79:
                    print("RIGHT")
                    inputThrust.vector[0] += constants.INPUT_FORCE

                elif event.scancode == 20:
                    print("TOGGLE DEBUG MODE")
                    constants.DEBUG_MODE = not(constants.DEBUG_MODE)

                elif event.scancode == 26:
                    print("TOGGLE DEBUG PANEL")
                    constants.DEBUG_PANEL = not(constants.DEBUG_PANEL)

                elif event.scancode == 4:
                    print("INCREASE TICK")
                    constants.TICK += 5
                    constants.TIME_STEP = constants.setTimeStep()

                elif event.scancode == 29:
                    print("DECREASE TICK")
                    constants.TICK -= 5 if constants.TICK > 5 else 0
                    constants.TIME_STEP = constants.setTimeStep()

                elif event.scancode == 22:
                    print("INCREASE SIM MULTIPLE")
                    constants.SIM_MULTIPLIER += 1
                    constants.TIME_STEP = constants.setTimeStep()

                elif event.scancode == 27:
                    print("DECREASE SIM MULTIPLE")
                    constants.SIM_MULTIPLIER -= 1 if constants.SIM_MULTIPLIER > 1 else 0
                    constants.TIME_STEP = constants.setTimeStep()

                elif event.scancode == 7:
                    print("INCREASE SPEED")
                    constants.SPEED += 0.1
                    constants.TIME_STEP = constants.setTimeStep()

                elif event.scancode == 6:
                    print("DECREASE SPEED")
                    constants.SPEED -= 0.1
                    constants.TIME_STEP = constants.setTimeStep()

                elif event.scancode == 10:
                    print("TOGGLE GRAVITY")
                    if constants.GRAVITY == 0:
                        constants.GRAVITY = 9.81
                    else:
                        constants.GRAVITY = 0

                elif event.scancode == 9:
                    print("TOGGLE FORCES")
                    constants.SHOW_FORCES = not(constants.SHOW_FORCES)

                else:
                    print(event.scancode)

            # CLICK TO GENERATE BALL WITH RANDOM COLOUR
            elif event.type == pygame.MOUSEBUTTONDOWN:
                print("CLICK")
                x, y = calc.pixelsToMetres(event.pos)

                r = 0.01 + constants.RANDOM_RADIUS()
                m = calc.circleArea(r)

                newBall = objects.Ball(id=objectCount, CoM=[x, y], radius=r, mass=m, colour=pygame.Color(constants.RANDOM_COLOUR()))
                screenObjects[newBall.id] = newBall
                objectCount += 1

            elif event.type == pygame.KEYUP:

                if event.scancode == 82:
                    print("UP_END")
                    inputThrust.vector[1] -= constants.INPUT_FORCE

                elif event.scancode == 81:
                    print("DOWN_END")
                    inputThrust.vector[1] += constants.INPUT_FORCE

                elif event.scancode == 80:
                    print("LEFT_END")
                    inputThrust.vector[0] += constants.INPUT_FORCE

                elif event.scancode == 79:
                    print("RIGHT_END")
                    inputThrust.vector[0] -= constants.INPUT_FORCE

            # QUIT
            if event.type == pygame.QUIT:
                running = False

        # RENDERER
        constants.screen.fill("white")

        # SIMULATION LOOP

        for i in range(0, constants.SIM_MULTIPLIER):

            constants.timeStamp += constants.TIME_STEP

            for objectId in screenObjects:

                screenObject = screenObjects[objectId]

                screenObject.getForces()

            # SIM ITERATION FUNCTIONs
            # eg - add torque to gear
            for func in constants.simIterationFunctions:
                func()

            # DETECT COLLISIONS AND RESOLVE OUTCOMES
            # also find contact forces
            collision.calcCollisions(screenObjects)

            # Get forces - store on object
                # thrusts
                # torques

            # Input forces
            avatar.thrusts.append(inputThrust)

            # Reset forces

            # RESOLVE CONTACTS
            if constants.ELASTICITY < 1:
                for cont in contacts.contactsToResolve:

                    cont.resolve()
                
            contacts.contactsToResolve = []


            for objectId in screenObjects:

                screenObject = screenObjects[objectId]

                screenObject.update()

                screenObject.reset()


        kineticEnergy = 0
        rotationalEnergy = 0
        gravitationalEnergy = 0

        for objectId in screenObjects:

            screenObject = screenObjects[objectId]

            if constants.DEBUG_PANEL & screenObject.checkCollisions & (screenObject.parent == None):

                kineticEnergy += screenObject.mass / 2 * dot(screenObject.velocity, screenObject.velocity)
                rotationalEnergy += (screenObject.MoI * screenObject.angularVelocity**2) / 2
                gravitationalEnergy += (screenObject.mass * (1000 + screenObject.CoM[1]) * constants.GRAVITY)

            screenObject.draw()
        
        frameRates.append(clock.get_fps())

        # DEBUG

        if constants.DEBUG_PANEL:

            panelContent = [
                f"time: {constants.timeStamp:.6f}",
                f"time step: {constants.TIME_STEP:.6f}",
                f"speed: {constants.SPEED: .2f}",
                f"sim multiple: {constants.SIM_MULTIPLIER}",
                f"target: {constants.TICK}",
                f"fps: {clock.get_fps():.2f}",
                f"objects: {len(screenObjects)}",
                f"kE: {kineticEnergy: .6f}",
                f"rE: {rotationalEnergy: .6f}",
                f"gE: {gravitationalEnergy: .6f}",
                f"totalE: {kineticEnergy + rotationalEnergy + gravitationalEnergy: .6f}"
            ]

            for i, content in enumerate(panelContent):

                text = font.render(content, True, "white", "black")
                textRect = text.get_rect()
                coordsPixels = calc.metresToPixels([1, 0.8])
                textRect.center = (coordsPixels[0], coordsPixels[1] + i * fontSize)
                constants.screen.blit(text, textRect)

        # REFRESH SCREEN
        
        pygame.display.flip()

        if (constants.timeStamp >= reportTime) & (reported == False):
            print(f"AVG FRAME RATE: {sum(frameRates)/len(frameRates): .2f}")
            reported = True

        # Capture the frame
        if rec:
            recorder.capture_frame(constants.screen)

    if rec:
        print("Saving recording...")
        recorder.end_recording()
        print("Recording Saved")
     
     
if __name__=="__main__":
    main(
        **dict(
            arg.split('=') for arg in sys.argv[1:]
        )
    )