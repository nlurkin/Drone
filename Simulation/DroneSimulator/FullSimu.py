from DroneSimulator.Body import Body
from DroneSimulator.matrix import *
from random import random
from scipy.constants.constants import pi
import matplotlib.pyplot as plt
import numpy

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
b.setConstants(I, K_d)
b.setParameters(0.38, 4.493)
b.setMotorConstants(Rho, K_v, K_t, K_tau, I_M, A_swept, A_xsec, Radius, C_D)

omega = [0,0,0]
omegaDot = [0,0,0]
a = [0,0,0]
thetaDot = [0,0,0]
theta = [0, 0, 0]
xDot = [0,0,0]
x = [0,0,40]

r = [random(),random(), random()] 
deviation = 1
thetaDot = vecScalarSum(-deviation, vecScalarProduct(2*deviation, r))
thetaDot[0] = 0
thetaDot[1] = 0

i = None
dt = 0.1
time = numpy.arange(0,5, dt)


for t in time:
    plotNbr = 1
    #set measurements
    b.setMeasure(xDot, omega)
    b.setMotorMeasure([0,0,0,0], [0,0,0,0]) #from controller decision
    #Get Input from arduino
    omega = thetadot2omega(thetaDot, theta)
    plt.figure(plotNbr)
    plt.plot(t, omega[0], 'rx')
    plt.plot(t, omega[1], 'gx')
    plt.plot(t, omega[2], 'bx')
    plotNbr+=1
    a = acceleration(theta, b)
    omegaDot = b.alpha()
    plt.figure(plotNbr)
    plt.plot(t, a[0], 'rx')
    plt.plot(t, a[1], 'gx')
    plt.plot(t, a[2], 'bx')
    plotNbr+=1
    omega = vecSum(omega, vecScalarProduct(dt, omegaDot))
    thetaDot = omega2thetadot(theta, omega)
    plt.figure(plotNbr)
    plt.plot(t, thetaDot[0], 'rx')
    plt.plot(t, thetaDot[1], 'gx')
    plt.plot(t, thetaDot[2], 'bx')
    plotNbr+=1
    theta = vecSum(theta, vecScalarProduct(dt, thetaDot))
    plt.figure(plotNbr)
    plt.plot(t, theta[0], 'rx')
    plt.plot(t, theta[1], 'gx')
    plt.plot(t, theta[2], 'bx')
    plotNbr+=1
    xDot = vecSum(xDot, vecScalarProduct(dt, a))
    plt.figure(plotNbr)
    plt.plot(t, xDot[0], 'rx')
    plt.plot(t, xDot[1], 'gx')
    plt.plot(t, xDot[2], 'bx')
    plotNbr+=1

    x = vecSum(x, vecScalarProduct(dt, xDot));
    plt.figure(plotNbr)
    plt.plot(t, x[0], 'rx')
    plt.plot(t, x[1], 'gx')
    plt.plot(t, x[2], 'bx')
    plotNbr+=1

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