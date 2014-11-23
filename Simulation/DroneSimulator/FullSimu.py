from random import random
import sys

import numpy
from numpy.ma.core import sin
from scipy.constants.constants import pi

from Body import Body
from ParamsClass import Params
from DroneMath.mathclasses import Vector, Quaternion
from sensorTest import PrecisionTest


from DroneMath.DronePlot import DronePlot
class Simu(object):
    Rho = 1.2250 #kg.m^-3
    #K_v = 3000 #rpm.V^-1
    K_v = 6.3E-5 #s.V.rad^-1
    #K_v = K_v*2*pi/60 #rad.s^-1.V^-1
    #K_t = K_v #N.m.A^-1
    K_t = 6.3E-3 #N.m.A^-1
    K_tau = 0.91 
    I_M = 104E-6 #
    Radius = 0.5 #m
    A_swept = pi*pow(Radius,2)
    A_xsec = A_swept
    C_D = 2
    
    I = Params.I
    K_d = 0.0013
    
    time = None
    dt = Params.dt
    
    t = None
    
    stepResponse = None
    
    waveResponse = None
    
    flipResponse = None
    
    moveTimes = None
    moveType = None
    
    pltpq = DronePlot()
    test = PrecisionTest(dt)
    
    def __init__(self):
        #self.b.setMotorConstants(self.Rho, self.K_v, self.K_t, self.K_tau, self.I_M, self.A_swept, self.A_xsec, self.Radius, self.C_D)
        self.time = numpy.arange(0,5, self.dt)
        self.t = 0
        self.gyro = Vector()
        self.angle = Vector()
        self.quat = Quaternion()
        self.stepResponse = [0, 0, 0]
        self.waveResponse = [0, 0, 0]
        self.flipResponse = [0, 0, 0]
        self.moveType = 0
        self.moveTimes = [0, 0, 0]
        
        self.b = Body(self)
               
    
    def initBody(self, local):
        self.b.setModel(self.I, self.K_d, Params.L, Params.Mass)
        self.b.setParameters(Params.TorqueIsSet, Params.MaxTorque, local)
        if local==True:
            self.b.initController()
        self.b.setMotorConstants(self.Rho, self.K_v, self.K_t, self.K_tau, self.I_M, self.A_swept, self.A_xsec, self.Radius, self.C_D)
    
    def deviate(self):
        r = Vector([random(),random(), random()]) 
        deviation = Params.MaxDeviation
        self.b.setOmega(-deviation+(2*deviation*r))
    
    def heavyside(self, t, t0):
        if (t-t0)<0:
            return 0
        else:
            return 1
        
    def moveStep(self):
        self.stepResponse[0] = self.heavyside(self.t, self.moveTimes[0])
        self.stepResponse[1] = self.heavyside(self.t, self.moveTimes[1])
        self.stepResponse[2] = self.heavyside(self.t, self.moveTimes[2])
        return self.stepResponse
            
    def moveWave(self):
        self.waveResponse[0] = 0.5*sin(self.t-self.moveTimes[0])*self.heavyside(self.t, self.moveTimes[0])
        self.waveResponse[1] = 0.5*sin(self.t-self.moveTimes[1])*self.heavyside(self.t, self.moveTimes[1])
        self.waveResponse[2] = 0.5*sin(self.t-self.moveTimes[2])*self.heavyside(self.t, self.moveTimes[2])
        return self.waveResponse
    
    def moveFlip(self):
        if self.t<self.moveTimes[0]:
            t = 0
        elif self.t<self.moveTimes[1]:
            t = -(self.t-self.moveTimes[0])
        elif self.t<self.moveTimes[2]:
            t = self.moveTimes[2]-self.t
        else:
            t = 0
        
        self.flipResponse[0] = t*pi
        return self.flipResponse
    
    def setMoveType(self, t):
        self.moveType = t
        if self.moveType==1:
            self.moveTimes[0] = self.t + Params.stepTimes[0]
            self.moveTimes[1] = self.t + Params.stepTimes[1]
            self.moveTimes[2] = self.t + Params.stepTimes[2]
        if self.moveType==2:
            self.moveTimes[0] = self.t + Params.waveTimes[0]
            self.moveTimes[1] = self.t + Params.waveTimes[1]
            self.moveTimes[2] = self.t + Params.waveTimes[2]
        if self.moveType==3:
            self.moveTimes[0] = self.t + Params.flipTimes[0]
            self.moveTimes[1] = self.t + Params.flipTimes[1]
            self.moveTimes[2] = self.t + Params.flipTimes[2]
    
    def getNextMove(self):
        val = None
        if self.moveType==0:
            val = [0, 0, 0]
        if self.moveType==1:
            val = self.moveStep()
        if self.moveType==2:
            val = self.moveWave()
        if self.moveType==3:
            val = self.moveFlip()
        
        return val
        
    
    def nextStep(self):
        self.singleStep(self.t)
        self.t = self.t+self.dt
    
    def singleStep(self, t):
        self.b.nextStep(self.dt)
        self.test.setAngularMeasure(self.b.Quat, self.b.Omega, self.t)
        self.test.setControl(self.b.CtrlInput)
        self.test.computeValues()
        self.test.compare(self.b.Omega, self.b.Alpha)
        self.plot(t)

    
    def mainLoop(self):
        for t in self.time:
            self.singleStep(t)
        self.plot()
    
    def plot(self, t):
       
        self.pltpq.addTime(t)
        self.pltpq.addQuaternion(self.b.Quat, self.b.ctrl.lastqRef)
        self.pltpq.addTheta(self.b.Angles)
        self.pltpq.addThetaDot(self.b.Omega)
        self.pltpq.addTorque(self.b.Torque, self.b.ctrl.Torque)
        self.pltpq.addX(self.b.Position)
        self.pltpq.addXDot(self.b.Velocity, self.b.ctrl.lastvRef)
        self.pltpq.addA(self.b.Acceleration)
        self.pltpq.addMotor(self.b.CtrlInput)
        self.pltpq.update()
        
    def getI(self):
        return self.I
    
    def getTheta(self):
        return self.theta
    
    def getTime(self):
        return self.t
    
    def getdTime(self):
        return self.dt
    
    def setRequiredTorque(self, t):
        self.b.setTorque(t)
        
    def setReference(self, ref, v):
        self.b.setReference(ref, v)
    
    def calibration(self):
        self.b.calibrate(self.dt)
        
if __name__ == "__main__":
    import matplotlib.pyplot as plt
    s = Simu()
    s.initBody(True)
    s.plotSetup()
    s.calibration()
    while(True):
        plt.pause(1)
    sys.exit(0);
