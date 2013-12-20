from Motor import Motor
from mathclasses import Vector


class Body(object):
    '''
    classdocs
    '''

    m1 = Motor()
    m2 = Motor()
    m3 = Motor()
    m4 = Motor()
    
    #calcule
    Torque = None #vector
    Thrust = None #vector
    Friction = None #vector
    Alpha = None #vector
    
    #parameters
    L = None #longueur des bras #scalar
    Mass = None #masse #scalar
    TorqueIsSet = None #switch #scalar
    
    #constants
    I = None #inertia matrix #vector
    K_d = None #friction-linear velocity #scalar
    
    maxTorque = 100
    
    #mesure
    Velocity = None #vector
    Omega = None #vector
    
    def __init__(self):
        '''
        Constructor
        '''
        self.m1.setParams(1)
        self.m2.setParams(-1)
        self.m3.setParams(1)
        self.m4.setParams(-1)
    
    def setConstants(self, I, K_d):
        self.I = I
        self.K_d = K_d
        
    def setParameters(self, L,  Mass, TorqueIsSet):
        self.L = L
        self.Mass = Mass
        self.TorqueIsSet = TorqueIsSet
    
    def setMeasure(self, Velocity, Omega):
        self.Velocity = Velocity
        self.Omega = Omega
    
    def setMotorMeasure(self, omega, alpha):
        self.m1.setMeasure(omega[0], alpha[0])
        self.m2.setMeasure(omega[1], alpha[1])
        self.m3.setMeasure(omega[2], alpha[2])
        self.m4.setMeasure(omega[3], alpha[3])    
        
    def setTorque(self, t):
        if t[0]>self.maxTorque:
            t[0] = self.maxTorque
        if t[1]>self.maxTorque:
            t[1] = self.maxTorque
        if t[2]>self.maxTorque:
            t[2] = self.maxTorque
        if t[0]<-self.maxTorque:
            t[0] = -self.maxTorque
        if t[1]<-self.maxTorque:
            t[1] = -self.maxTorque
        if t[2]<-self.maxTorque:
            t[2] = -self.maxTorque
        self.Torque = t
        print "Setting torque to"
        print self.Torque
    
    def setMotorConstants(self, Rho, K_v, K_t, K_tau, I_M, A_swept, A_xsec, Radius, C_D):
        self.m1.setConstants(Rho, K_v, K_t, K_tau, I_M, A_swept, A_xsec, Radius, C_D)
        self.m2.setConstants(Rho, K_v, K_t, K_tau, I_M, A_swept, A_xsec, Radius, C_D)
        self.m3.setConstants(Rho, K_v, K_t, K_tau, I_M, A_swept, A_xsec, Radius, C_D)
        self.m4.setConstants(Rho, K_v, K_t, K_tau, I_M, A_swept, A_xsec, Radius, C_D)
    
    def torque(self):
        if not self.TorqueIsSet:
            self.Torque = [self.L*(self.m1.thrust()-self.m3.thrust()), self.L*(self.m2.thrust()-self.m4.thrust()), self.m1.tau_z()+self.m2.tau_z()+self.m3.tau_z()+self.m4.tau_z()]
        return self.Torque
    
    def thrust(self):
        self.Thrust = [0, 0, self.m1.thrust()+self.m2.thrust()+self.m3.thrust()+self.m4.thrust()]
        return self.Thrust
    
    def friction(self):
        self.Friction = vecScalarProduct(-self.K_d, self.Velocity)
        return self.Friction
    
    def alpha(self):
        #print "Omega"
        #print self.Omega
        self.Alpha = [(self.torque()[0] - (self.I[1]-self.I[2])*self.Omega[1]*self.Omega[2])/self.I[0],
                      (self.torque()[1] - (self.I[2]-self.I[0])*self.Omega[0]*self.Omega[2])/self.I[1],
                      (self.torque()[2] - (self.I[0]-self.I[1])*self.Omega[0]*self.Omega[1])/self.I[2]]
        #self.Alpha = [self.torque()[0],
        #              self.torque()[1],
        #              self.torque()[2]]
        #print "Alpha"
        #print self.Alpha
        return self.Alpha
        
    def mass(self):
        return self.Mass
    