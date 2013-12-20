from Body import Body
from mathclasses import Quaternion
from scipy.constants.constants import pi
import numpy
    
class Simu(object):
    Rho = 1.2250 #kg.m^-3
    K_v = 3000 #rpm.V^-1
    K_v = K_v*2*pi/60 #rad.s^-1.V^-1
    #K_t = K_v #N.m.A^-1
    K_t = 0.00308 #N.m.A^-1
    K_tau = 0.91 
    I_M = 104*10^-6 #
    Radius = 0.1 #m
    A_swept = pi*pow(Radius,2)
    A_xsec = A_swept
    C_D = 0.3
    
    #I = [0.177, 0.177, 0.334]
    I = [0.334, 0.334, 0.334]
    K_d = 0.0013
    b = Body()
    
    omega = None
    omegaDot = None
    a = None
    thetaDot = None
    theta = None
    xDot = None
    x = None
    time = None
    dt = None
    
    t = None
    
    gyro = None
    angle = None
    quat = None
    
    stepResponse = None
    stepTimes = [0.5, 2.5, 4.5]
    
    waveResponse = None
    waveTimes = [0.5, 1, 1.5]
    
    flipResponse = None
    flipTimes = [1, 2, 3]
    
    moveTimes = None
    moveType = None
    
    def __init__(self):
        self.b.setConstants(self.I, self.K_d)
        self.b.setParameters(0.38, 4.493, True)
        self.b.setMotorConstants(self.Rho, self.K_v, self.K_t, self.K_tau, self.I_M, self.A_swept, self.A_xsec, self.Radius, self.C_D)
        self.omega = [0,0,0]
        self.omegaDot = [0,0,0]
        self.a = [0,0,0]
        self.thetaDot = [0,0,0]
        self.theta = [0, 0, 0]
        self.xDot = [0,0,0]
        self.x = [0,0,40]
        self.dt = 0.1
        self.time = numpy.arange(0,5, self.dt)
        self.t = 0
        self.gyro = [0, 0, 0]
        self.angle = [0, 0, 0]
        self.quat = Quaternion([self.theta[2], self.theta[1], self.theta[0]])

    def getQuaternion(self):
        return self.quat
    
    def getGyro(self):
        return self.gyro
    
    def getAcc(self):
        return self.a