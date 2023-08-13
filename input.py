import constants

def updateSimMultiplier(delta):
    constants.SIM_MULTIPLIER += delta
    setGeneralDampingF(constants.GENERAL_DAMPING)

def setGeneralDampingF(newFactor):
    constants.GENERAL_DAMPING = newFactor
    constants.GENERAL_DAMPING_F = 1 - constants.GENERAL_DAMPING / constants.SIM_MULTIPLIER
    print(constants.GENERAL_DAMPING)
    print(constants.GENERAL_DAMPING_F)
