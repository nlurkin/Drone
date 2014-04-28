'''
Created on 27 Apr 2014

@author: Nicolas
'''

import random
import matplotlib.pyplot as plt

from Kalman import KalmanGeneric


def test1():
    kal = KalmanGeneric(1, 2, 1)
    
    dt = 0.1
    sigma_theta = 0.01
    
    kal.setMatH([[1,0]])
    kal.setMatF([[1, dt],[0, 1]])
    kal.setQ([[dt*dt*dt*dt/4., dt*dt*dt/2.], [dt*dt*dt/2., dt*dt]])
    kal.setR([[sigma_theta*sigma_theta]])
    kal.setB([[dt*dt/2.],[dt]])
    
    theta = 0
    omega = 0
    #alpha = 0.2
    
    kal.setX0([[0],[0]])
    kal.setP0([[1000, 0], [0, 1000]])
    
    av = []
    thv = []
    ov = []
    tkv = []
    okv = []
    tv = []
    
    for t in range(0, 500):
        
        
        alpha = random.normalvariate(0, 1) + 10;
        omega = omega + alpha*dt
        theta = theta + omega*dt
        [[t_k], [v_k]] = kal.newMeasure([[random.normalvariate(theta, sigma_theta)]],[[10]])
        
        tv.append(t*dt)
        av.append(alpha)
        ov.append(omega)
        thv.append(theta)
        tkv.append(t_k - theta)
        okv.append(v_k - omega)
        #print "Real value %s / %s" % (theta, omega)
        #print "           %s / %s" % (t_k, v_k)
    
    #print kal
    
    plt.subplot(3, 1, 1)
    plt.plot(tv, tkv, "r")#, tv, thv, "b")
    plt.subplot(3, 1, 2)
    plt.plot(tv, okv, "r")#, tv, ov, "b")
    plt.subplot(3, 1, 3)
    plt.plot(tv, av, "b")
    
    plt.show()
    #while True:
    #    plt.pause(1)

def test2():
    kal = KalmanGeneric(1,2, 0)
    
    dt = 0.1
    sigma_omega = 0.01
    
    kal.setMatH([[1,0]])
    kal.setMatF([[1, dt],[0, 1]])
    kal.setQ([[dt*dt, 0], [0, 1.]])
    kal.setR([[sigma_omega*sigma_omega]])
    #kal.setB([[0],[0]])
    
    omega = 1
    alpha = 0.5
    
    kal.setX0([[1],[0]])
    kal.setP0([[1000, 0], [0, 1]])
    
    av = []
    ov = []
    okv = []
    akv = []
    tv = []
    delta_akv = []
    
    for t in range(0, 500):
        alpha = alpha + 0.1
        omega = omega + alpha*dt
        [o_k, a_k] = kal.newMeasure([[random.normalvariate(omega, sigma_omega)]],[[]])
        
        tv.append(t*dt)
        av.append(alpha)
        ov.append(omega)
        akv.append(a_k[0])
        okv.append(o_k[0])
        delta_akv.append(alpha-a_k[0])
        #print "Real value %s / %s" % (theta, omega)
        #print "           %s / %s" % (t_k, v_k)
    
    #print kal
    
    plt.subplot(3, 1, 1)
    plt.plot(tv, okv, "r", tv, ov, "b")
    plt.subplot(3, 1, 2)
    plt.plot(tv, akv, "r", tv, av, "b")
    plt.subplot(3, 1, 3)
    plt.plot(tv, delta_akv, "r")
    
    plt.show()
    
if __name__ == "__main__":
    test1()
        