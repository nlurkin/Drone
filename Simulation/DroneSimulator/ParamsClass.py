from mathclasses import Vector

class Params:
    '''
    classdocs
    '''

    runLocally = True
    ctrlType = 1
    
    I = Vector([0.177, 0.177, 0.334])
    #I = [0.177, 0.177, 0.334]
    L = 0.38
    Mass = 4.493
    
    stepTimes = [0.5, 4.5, 8.5]
    waveTimes = [0.5, 1, 1.5]
    flipTimes = [1, 2, 3]
    
    dt = 0.1
    
    moveType = 3
    MaxTorque = 5
    MaxDeviation = 5
    def __init__(self):
        '''
        Constructor
        '''
        