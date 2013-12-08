from Body import Body
from matrix import *
from random import random
from scipy.constants.constants import pi
import matplotlib.pyplot as plt
import numpy
import qmath
    
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
    
    I = [0.177, 0.177, 0.334]
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
        self.quat = qmath.quaternion    
    
    def deviate(self):
        r = [random(),random(), random()] 
        deviation = 10
        self.thetaDot = vecScalarSum(-deviation, vecScalarProduct(2*deviation, r))
        self.thetaDot[0] = 0
        self.thetaDot[1] = 0
        print self.thetaDot
    
    def nextStep(self):
        self.singleStep(self.t)
        self.t = self.t+self.dt
    
    def singleStep(self, t):
        #set measurements
        self.b.setMeasure(self.xDot, self.omega)
        self.b.setMotorMeasure([0,0,0,0], [0,0,0,0]) #from controller decision
        #Get Input from arduino
        self.omega = thetadot2omega(self.thetaDot, self.theta)
        self.a = acceleration(self.theta, self.b)
        self.omegaDot = self.b.alpha()
        self.omega = vecSum(self.omega, vecScalarProduct(self.dt, self.omegaDot))
        self.thetaDot = omega2thetadot(self.theta, self.omega)
        self.theta = vecSum(self.theta, vecScalarProduct(self.dt, self.thetaDot))
        self.xDot = vecSum(self.xDot, vecScalarProduct(self.dt, self.a))
        self.x = vecSum(self.x, vecScalarProduct(self.dt, self.xDot));
        
        self.buildAttitude()
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
        plt.plot(self.t, self.quat[1], 'rx')
        plt.plot(self.t, self.quat[2], 'gx')
        plt.plot(self.t, self.quat[3], 'bx')
        plt.subplot(2, 2 ,2)
        #theta
        plt.plot(t, self.theta[0], 'rx')
        plt.plot(t, self.theta[1], 'gx')
        plt.plot(t, self.theta[2], 'bx')
        plt.subplot(2, 2 ,3)
        #thetadot
        plt.plot(t, self.thetaDot[0], 'rx')
        plt.plot(t, self.thetaDot[1], 'gx')
        plt.plot(t, self.thetaDot[2], 'bx')
        plt.subplot(2, 2 ,4)
        #torque
        plt.plot(t, self.b.Torque[0], 'rx')
        plt.plot(t, self.b.Torque[1], 'gx')
        plt.plot(t, self.b.Torque[2], 'bx')
        
        plotNbr+=1
        
        plt.figure(plotNbr)
        plt.plot(t, self.omega[0], 'rx')
        plt.plot(t, self.omega[1], 'gx')
        plt.plot(t, self.omega[2], 'bx')
        plotNbr+=1
        
        plt.figure(plotNbr)
        plt.plot(t, self.a[0], 'rx')
        plt.plot(t, self.a[1], 'gx')
        plt.plot(t, self.a[2], 'bx')
        plotNbr+=1
        
        plt.figure(plotNbr)
        plt.plot(t, self.xDot[0], 'rx')
        plt.plot(t, self.xDot[1], 'gx')
        plt.plot(t, self.xDot[2], 'bx')
        plotNbr+=1
        
        plt.figure(plotNbr)
        plt.plot(t, self.x[0], 'rx')
        plt.plot(t, self.x[1], 'gx')
        plt.plot(t, self.x[2], 'bx')
        plotNbr+=1
        
        
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
        plt.title("omega")
        plt.grid(True)
        plotNbr+=1
        
        plt.figure(plotNbr)
        plt.title("a")
        plt.grid(True)
        plotNbr+=1
        
        plt.figure(plotNbr)
        plt.title("xDot")
        plt.grid(True)
        plotNbr+=1
        
        plt.figure(plotNbr)
        plt.title("x")
        plt.grid(True)
        plotNbr+=1

        
        #plt.draw()
    
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
    
    def buildAttitude(self):
        print self.theta
        print self.angle
        print self.dt
        print self.quat
        self.gyro = [(self.theta[0]-self.angle[0])/self.dt, (self.theta[1]-self.angle[1])/self.dt, (self.theta[2]-self.angle[2])/self.dt]
        self.angle = self.theta
        self.quat = qmath.quaternion([self.theta[2], self.theta[1], self.theta[0]])

    def getQuaternion(self):
        return self.quat
    
    def getGyro(self):
        return self.gyro
    
    def getAcc(self):
        return self.a