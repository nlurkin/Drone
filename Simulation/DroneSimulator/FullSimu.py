from Body import Body
from ParamsClass import Params
from mathclasses import Vector, Quaternion
from numpy.ma.core import sin
from random import random
from scipy.constants.constants import pi
import matplotlib.pyplot as plt
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
    
    I = Params.I
    K_d = 0.0013
    b = Body()
    
    time = None
    dt = Params.dt
    
    t = None
    
    stepResponse = None
    
    waveResponse = None
    
    flipResponse = None
    
    moveTimes = None
    moveType = None
    
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
    
    def initBody(self, local):
        self.b.setModel(self.I, self.K_d, Params.L, Params.Mass)
        self.b.setParameters(True, Params.MaxTorque, local)
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
        
        plt.figure(50)
        plt.subplot(3, 1 ,1)
        plt.plot(self.t, val[0], 'ro')
        plt.subplot(3, 1 ,2)
        plt.plot(self.t, val[1], 'go')
        plt.subplot(3, 1 ,3)
        plt.plot(self.t, val[2], 'bo')
        return val
        
    
    def nextStep(self):
        self.singleStep(self.t)
        self.t = self.t+self.dt
    
    def singleStep(self, t):
        #set measurements
        #self.b.setMotorMeasure([0,0,0,0], [0,0,0,0]) #from controller decision
        #Get Input from arduino
        self.b.nextStep(self.dt)
        self.plot(t)

    
    def mainLoop(self):
        for t in self.time:
            self.singleStep(t)
        self.plot()
    
    def plot(self, t):
        plotNbr=1
        plt.figure(plotNbr)
        plt.subplot(2, 2 ,1)
        #quaternion
        plt.plot(self.t, self.b.Quat.x, 'rx')
        plt.plot(self.t, self.b.Quat.y, 'gx')
        plt.plot(self.t, self.b.Quat.z, 'bx')
        plt.plot(self.t, self.b.ctrl.QRef.x, 'ro')
        plt.plot(self.t, self.b.ctrl.QRef.y, 'go')
        plt.plot(self.t, self.b.ctrl.QRef.z, 'bo')

        plt.subplot(2, 2 ,2)
        #theta
        plt.plot(t, self.b.Angles[0], 'rx')
        plt.plot(t, self.b.Angles[1], 'gx')
        plt.plot(t, self.b.Angles[2], 'bx')
        plt.subplot(2, 2 ,3)
        #thetadot
        plt.plot(t, self.b.Omega[0], 'rx')
        plt.plot(t, self.b.Omega[1], 'gx')
        plt.plot(t, self.b.Omega[2], 'bx')
        plt.subplot(2, 2 ,4)
        #torque
        plt.plot(t, self.b.Torque[0], 'rx')
        plt.plot(t, self.b.Torque[1], 'gx')
        plt.plot(t, self.b.Torque[2], 'bx')
        

        plotNbr+=1
        plt.figure(plotNbr)
        plt.subplot(2, 2 ,1)
        plt.plot(t, self.b.Position[0], 'rx')
        plt.plot(t, self.b.Position[1], 'gx')
        plt.plot(t, self.b.Position[2], 'bx')
        plt.subplot(2, 2 ,2)
        plt.plot(t, self.b.Velocity[0], 'rx')
        plt.plot(t, self.b.Velocity[1], 'gx')
        plt.plot(t, self.b.Velocity[2], 'bx')
        plt.subplot(2, 2 ,3)
        plt.plot(t, self.b.Acceleration[0], 'rx')
        plt.plot(t, self.b.Acceleration[1], 'gx')
        plt.plot(t, self.b.Acceleration[2], 'bx')
        plt.subplot(2, 2 ,4)
        
        plotNbr+=1
        plt.figure(50)
        plt.subplot(3, 1 ,1)
        plt.plot(t, self.b.Angles[0], 'rx')
        plt.subplot(3, 1 ,2)
        plt.plot(t, self.b.Angles[1], 'gx')
        plt.subplot(3, 1 ,3)
        plt.plot(t, self.b.Angles[2], 'bx')
        
        
        
        
    def plotSetup(self):
        plotNbr=1
        plt.figure(plotNbr)
        plt.subplot(2, 2 ,1)
        plt.title("quaternion")
        plt.grid(True)
        #quaternion
        plt.subplot(2, 2 ,2)
        #theta
        plt.title("theta")
        plt.grid(True)
        plt.subplot(2, 2 ,3)
        #thetadot
        plt.title("thetaDot")
        plt.grid(True)
        plt.subplot(2, 2 ,4)
        #torque
        plt.title("torque")
        plt.grid(True)
        
        plotNbr+=1

        plt.figure(plotNbr)
        plt.subplot(2, 2 ,1)
        plt.title("x")
        plt.grid(True)
        plt.subplot(2, 2 ,2)
        plt.title("xDot")
        plt.grid(True)
        plt.subplot(2, 2 ,3)
        plt.title("a")
        plt.grid(True)
        plt.subplot(2, 2 ,4)
        plt.figure(plotNbr)
        plt.title("omega")
        plt.grid(True)
        
        plotNbr+=1
        plt.figure(50)
        plt.subplot(3, 1 ,1)
        plt.title("MoveX")
        plt.grid(True)
        plt.subplot(3, 1 ,2)
        plt.title("MoveY")
        plt.grid(True)
        plt.subplot(3, 1 ,3)
        plt.title("MoveZ")
        plt.grid(True)

    
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
        
    def setReference(self, ref):
        self.b.setReference(ref)
        