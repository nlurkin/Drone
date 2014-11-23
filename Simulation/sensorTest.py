'''
Created on 28 Apr 2014

@author: Nicolas
'''

import matplotlib.pyplot as plt
from DroneMath.Kalman import KalmanGeneric

class PrecisionTest(object):
    '''
    classdocs
    '''
    
    position = None
    speed = None
    acceleration = None
    
    attitude = None
    omega = None
    alpha = None
    
    control = None
    
    time = None
    
    old_attitude = None
    old_omega = None
    old_time = None
    
    v_time = None
    
    true_omega = None
    v_omega = None
    diff_omega = None
    
    true_alpha = None
    v_alpha = None
    diff_alpha = None
    
    type = 2
    
    filter = None
    
    queue_alpha = None
    size_alpha = 2
    
    def __init__(self, dt):
        '''
        Constructor
        '''
        
        self.v_time = []
        
        self.true_omega = []
        self.v_omega =[]
        self.diff_omega = []

        self.true_alpha = []
        self.v_alpha =[]
        self.diff_alpha = []
        
        if self.type==1:
            self.queue_alpha = []
        elif self.type==2:
            self.filter = KalmanGeneric(1, 2, 1)
            #self.filter.setMatH([[1,0]])
            #self.filter.setMatF([[1, dt],[0, 1]])
            #self.filter.setQ([[dt*dt, 0], [0, 1.]])
            #self.filter.setR([[1.86E-5, 0], [0, 1.87E-5]])
            sigma_omega = 1./(131.*2.)
            self.filter.setMatH([[1,0]])
            self.filter.setMatF([[1, dt],[0, 1]])
            self.filter.setQ([[dt*dt*dt*dt/4., dt*dt*dt/2.], [dt*dt*dt/2., dt*dt]])
            self.filter.setR([[sigma_omega*sigma_omega]])
            self.filter.setB([[dt],[1]])
            self.filter.setX0([[0],[0]])
            self.filter.setP0([[1, 0], [0, 1]])

    
    def setAngularMeasure(self, quat, omega, t):
        self.old_attitude = self.attitude
        self.old_omega = self.omega
        self.old_time = self.time
        
        self.attitude = quat
        self.omega = int(omega[0]*(131*2))/(131.*2.)
        #print "%s / %s" % (self.omega, omega[0])
        self.time = t
    
    def setControl(self, c):
        self.control = c
    
    def computeValues(self):
        if self.type==0:
            self.computeSingleAverage()
        elif self.type==1:
            self.computeMultipleAverage()
        elif self.type==2:
            self.computeKalman()
        
    def computeSingleAverage(self):
        if(self.old_time==None):
            return
        self.alpha = (self.omega - self.old_omega)/(self.time-self.old_time)
    
    def computeMultipleAverage(self):
        if(self.old_time==None):
            return
        alpha = (self.omega - self.old_omega)/(self.time-self.old_time)
        if len(self.queue_alpha)>self.size_alpha:
            self.queue_alpha.pop()
        self.queue_alpha.insert(0, alpha)
        self.alpha = 0.
        for el in self.queue_alpha:
            self.alpha += el
        
        self.alpha /= len(self.queue_alpha)
    
    def computeKalman(self):
        if(self.old_time==None):
            self.filter.setX0([[self.omega],[0]])
            self.filter.setP0([[1000,0], [0,1]])
        [[self.omega],[self.alpha]] = self.filter.newMeasure([[self.omega]], [[0]])
        #print self.omega, self.alpha

    def compare(self, omega, alpha):
        if(self.old_time==None):
            return
        self.v_time.append(self.time)

        self.true_omega.append(omega[0])
        self.v_omega.append(self.omega)
        self.diff_omega.append(self.omega-omega[0])

        self.true_alpha.append(alpha[0])
        self.v_alpha.append(self.alpha)
        self.diff_alpha.append(self.alpha-alpha[0])
         
    def plot(self):
        plt.subplot(4, 1, 1)
        plt.title("omega")
        plt.plot(self.v_time, self.v_omega, "r", self.v_time, self.true_omega, "b")
        plt.subplot(4, 1, 2)
        plt.title("omega_diff")
        plt.plot(self.v_time, self.diff_omega, "r")

        plt.subplot(4, 1, 3)
        plt.title("alpha")
        plt.plot(self.v_time, self.v_alpha, "r", self.v_time, self.true_alpha, "b")
        plt.subplot(4, 1, 4)
        plt.title("alpha_diff")
        plt.plot(self.v_time, self.diff_alpha, "r")
        
        plt.show()