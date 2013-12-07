from Body import Body
from matrix import *
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
    
    def __init__(self):
        self.b.setConstants(self.I, self.K_d)
        self.b.setParameters(0.38, 4.493)
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
    
    def deviate(self):
        r = [random(),random(), random()] 
        deviation = 1
        self.thetaDot = vecScalarSum(-deviation, vecScalarProduct(2*deviation, r))
        self.thetaDot[0] = 0
        self.thetaDot[1] = 0
    
    def singleStep(self, t):
        plotNbr = 1
        #set measurements
        self.b.setMeasure(self.xDot, self.omega)
        self.b.setMotorMeasure([0,0,0,0], [0,0,0,0]) #from controller decision
        #Get Input from arduino
        omega = thetadot2omega(self.thetaDot, self.theta)
        plt.figure(plotNbr)
        plt.plot(t, self.omega[0], 'rx')
        plt.plot(t, self.omega[1], 'gx')
        plt.plot(t, self.omega[2], 'bx')
        plotNbr+=1
        self.a = acceleration(self.theta, self.b)
        self.omegaDot = self.b.alpha()
        plt.figure(plotNbr)
        plt.plot(t, self.a[0], 'rx')
        plt.plot(t, self.a[1], 'gx')
        plt.plot(t, self.a[2], 'bx')
        plotNbr+=1
        self.omega = vecSum(self.omega, vecScalarProduct(self.dt, self.omegaDot))
        self.thetaDot = omega2thetadot(self.theta, self.omega)
        plt.figure(plotNbr)
        plt.plot(t, self.thetaDot[0], 'rx')
        plt.plot(t, self.thetaDot[1], 'gx')
        plt.plot(t, self.thetaDot[2], 'bx')
        plotNbr+=1
        self.theta = vecSum(self.theta, vecScalarProduct(self.dt, self.thetaDot))
        plt.figure(plotNbr)
        plt.plot(t, self.theta[0], 'rx')
        plt.plot(t, self.theta[1], 'gx')
        plt.plot(t, self.theta[2], 'bx')
        plotNbr+=1
        self.xDot = vecSum(self.xDot, vecScalarProduct(self.dt, self.a))
        plt.figure(plotNbr)
        plt.plot(t, self.xDot[0], 'rx')
        plt.plot(t, self.xDot[1], 'gx')
        plt.plot(t, self.xDot[2], 'bx')
        plotNbr+=1
        self.x = vecSum(self.x, vecScalarProduct(self.dt, self.xDot));
        plt.figure(plotNbr)
        plt.plot(t, self.x[0], 'rx')
        plt.plot(t, self.x[1], 'gx')
        plt.plot(t, self.x[2], 'bx')
        plotNbr+=1
    
    def mainLoop(self):
        i = None
        for t in self.time:
            self.singleStep(t)
        
        plotNbr=1
        plt.figure(plotNbr)
        plt.title("omega")
        plt.grid(True)
        plotNbr+=1
        
        plt.figure(plotNbr)
        plt.title("a")
        plt.grid(True)
        plotNbr+=1
        
        plt.figure(plotNbr)
        plt.title("thetaDot")
        plt.grid(True)
        plotNbr+=1
        
        plt.figure(plotNbr)
        plt.title("theta")
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
        
        plt.show()
    
    def getI(self):
        return self.I