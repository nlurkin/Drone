from Controller import PID
from mathclasses import Matrix
from mathclasses import Quaternion, Vector
from numpy.ma.core import sin, tan, cos


class Body(object):
    '''
    classdocs
    '''

    '''m1 = Motor()
    m2 = Motor()
    m3 = Motor()
    m4 = Motor()'''
    
    '''internal state:
    alpha    -> computed from Torque
    omega    -> Sensor(gyro)
    quat    -> Sensor(DMZ)
    acceleration    -> Sensor(acc)
    velocity
    position
    '''
    
    ctrl = PID()
    
    #set by controller
    Torque = Vector() #vector
    
    #from motor
    Thrust = Vector() #vector
    
    #from physics
    Friction = Vector() #vector
    
    #state
    Alpha = Vector() #vector
    Omega = Vector() #vector
    Quat = Quaternion() #quaternion
    ThetaDot = Vector()
    Angles = Vector()
    Acceleration = Vector() #vector
    Velocity = Vector() #vector
    Position = Vector() #vector
    
    #model parameters
    L = None #longueur des bras #scalar
    Mass = None #masse #scalar
    I = Vector() #inertia matrix #vector
    K_d = None #friction-linear velocity #scalar
    
    #simulation parameters
    TorqueIsSet = False #switch #scalar
    MaxTorque = 100
    UseController = False
    
    def __init__(self):
        '''
        Constructor
        '''
        '''self.m1.setParams(1)
        self.m2.setParams(-1)
        self.m3.setParams(1)
        self.m4.setParams(-1)'''
    
    def setModel(self, I, K_d, L, Mass):
        self.I = I
        self.K_d = K_d
        self.L = L
        self.Mass = Mass
        
    def setParameters(self,TorqueIsSet, MaxTorque, UseController):
        self.TorqueIsSet = TorqueIsSet
        self.MaxTorque = MaxTorque 
        self.UseController = UseController
    
    #def setMotorMeasure(self, omega, alpha):
        '''self.m1.setMeasure(omega[0], alpha[0])
        self.m2.setMeasure(omega[1], alpha[1])
        self.m3.setMeasure(omega[2], alpha[2])
        self.m4.setMeasure(omega[3], alpha[3])'''    
    
    def setReference(self, qRef, aRef):
        self.ctrl.setQRef(qRef, aRef)
    
    def initController(self):
        self.ctrl.setI(self.I)
        self.ctrl.setPs([10,15,2])
    
    def applyController(self):
        t = self.ctrl.computePP(self.Quat, self.Omega, self.Angles)
        if t[0]>self.MaxTorque:
            t[0] = self.MaxTorque
        if t[1]>self.MaxTorque:
            t[1] = self.MaxTorque
        if t[2]>self.MaxTorque:
            t[2] = self.MaxTorque
        if t[0]<-self.MaxTorque:
            t[0] = -self.MaxTorque
        if t[1]<-self.MaxTorque:
            t[1] = -self.MaxTorque
        if t[2]<-self.MaxTorque:
            t[2] = -self.MaxTorque
        self.Torque = t
        print "Setting torque to" + self.Torque.__str__()
    
    def nextStep(self, dt):
        if self.UseController:
            self.applyController()
        
        #alpha
        self.computeAlpha(dt)
        #omega
        self.computeOmega(dt)
        #thetaDot
        self.computeThetaDot(dt)
        #theta
        self.computeAngles(dt)
        #quaternion
        self.computeQuaternion(dt)
                
        #acceleration
        #vitesse
        #position
        
    def setMotorConstants(self, Rho, K_v, K_t, K_tau, I_M, A_swept, A_xsec, Radius, C_D):
        '''self.m1.setConstants(Rho, K_v, K_t, K_tau, I_M, A_swept, A_xsec, Radius, C_D)
        self.m2.setConstants(Rho, K_v, K_t, K_tau, I_M, A_swept, A_xsec, Radius, C_D)
        self.m3.setConstants(Rho, K_v, K_t, K_tau, I_M, A_swept, A_xsec, Radius, C_D)
        self.m4.setConstants(Rho, K_v, K_t, K_tau, I_M, A_swept, A_xsec, Radius, C_D)'''
    
    def torque(self):
        #if not self.TorqueIsSet:
            #self.Torque = [self.L*(self.m1.thrust()-self.m3.thrust()), self.L*(self.m2.thrust()-self.m4.thrust()), self.m1.tau_z()+self.m2.tau_z()+self.m3.tau_z()+self.m4.tau_z()]
        return self.Torque
    
    def thrust(self):
        #self.Thrust = Vector([0, 0, self.m1.thrust()+self.m2.thrust()+self.m3.thrust()+self.m4.thrust()])
        return self.Thrust
    
    def friction(self):        
        self.Friction = -self.Velocity*self.K_d
        return self.Friction
    
    def computeAlpha(self, dt):
        self.Alpha = Vector([(self.torque()[0] - (self.I[1]-self.I[2])*self.Omega[1]*self.Omega[2])/self.I[0],
                      (self.torque()[1] - (self.I[2]-self.I[0])*self.Omega[0]*self.Omega[2])/self.I[1],
                      (self.torque()[2] - (self.I[0]-self.I[1])*self.Omega[0]*self.Omega[1])/self.I[2]])
        '''self.Alpha = Vector([(self.torque()[0])/self.I[0],
                      (self.torque()[1])/self.I[1],
                      (self.torque()[2])/self.I[2]])'''
    
    def computeOmega(self, dt):
        self.Omega = self.Omega + self.Alpha*dt
    
    def computeThetaDot(self, dt):
        mat1 = Matrix([[1, sin(self.Angles[0])*tan(self.Angles[1]), cos(self.Angles[0])*tan(self.Angles[1])], 
                       [0, cos(self.Angles[0]), -sin(self.Angles[0])], 
                       [0, sin(self.Angles[0])/cos(self.Angles[1]), cos(self.Angles[0])/cos(self.Angles[1])]])
        print mat1
        print self.Omega
        self.ThetaDot = mat1*self.Omega
        print self.ThetaDot

    def computeAngles(self, dt):
        print "ThetaDot " + self.ThetaDot
        print "Old angle " + self.Angles
        self.Angles = self.Angles + self.ThetaDot*dt
        print "New angle " + self.Angles 
        
    def computeQuaternion(self, dt):
        self.Quat = Quaternion([self.Angles[0], self.Angles[1], self.Angles[2]])
        
    def mass(self):
        return self.Mass
    
    def setOmega(self, v):
        self.Omega = v
        #self.Omega[1] = 0
        #self.Omega[0] = 0