from DroneMath.mathclasses import Vector

class Params:
    '''
    classdocs
    '''

    comPort = 3
    runLocally = True
    ground = False
    ctrlType = 1
    
    I = Vector([0.177, 0.177, 0.334])
    #I = [0.177, 0.177, 0.334]
    L = 0.38
    Mass = 1.493
    
    stepTimes = [0.5, 4.5, 8.5]
    waveTimes = [0.5, 1, 1.5]
    flipTimes = [1, 2, 3]
    
    dt = 0.001
    
    moveType = 1
    MaxTorque = 5
    MaxDeviation = 5
    
    serialSleep = 0.01
    
    MaxOmega = 500
    Gravity = Vector([0, 0, -9.81])
    
    TorqueIsSet = False
    
    PCoeff = [20,4, 0]
    
    def __init__(self):
        '''
        Constructor
        '''
        