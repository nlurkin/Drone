from Controller import PID, PSquare
from Motor import Motor
from ParamsClass import Params
from mathclasses import Matrix, Quaternion, Vector
from numpy.ma.core import sin, tan, cos


class Body(object):
    '''
    classdocs
    '''

    m1 = Motor()
    m2 = Motor()
    m3 = Motor()
    m4 = Motor()
    
    '''internal state:
    alpha    -> computed from Torque
    omega    -> Sensor(gyro)
    quat    -> Sensor(DMZ)
    acceleration    -> Sensor(acc)
    velocity
    position
    '''
    
    ctrl = None
    
    #set by controller
    CtrlInput = []
    
    #from motor
    Torque = Vector() #vector
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
        if Params.ctrlType==0:
            self.ctrl = PID()
            print "Using controller PID"
        elif Params.ctrlType==1:
            self.ctrl = PSquare()
            print "Using controller PSquare"
        self.m1.setParams(1)
        self.m2.setParams(-1)
        self.m3.setParams(1)
        self.m4.setParams(-1)
    
    def setModel(self, I, K_d, L, Mass):
        self.I = I
        self.K_d = K_d
        self.L = L
        self.Mass = Mass
        
    def setParameters(self,TorqueIsSet, MaxTorque, UseController):
        self.TorqueIsSet = TorqueIsSet
        self.MaxTorque = MaxTorque 
        self.UseController = UseController
    
    def setMotorMeasure(self, omega, alpha):
        self.m1.setMeasure(omega[0], alpha[0])
        self.m2.setMeasure(omega[1], alpha[1])
        self.m3.setMeasure(omega[2], alpha[2])
        self.m4.setMeasure(omega[3], alpha[3])    
    
    def setReference(self, qRef):
        self.ctrl.setQRef(qRef)
    
    def initController(self):
        self.ctrl.setI(self.I)
        if Params.ctrlType==0:
            self.ctrl.setPs([10,15,2])
        elif Params.ctrlType==1:
            self.ctrl.setPs([20,4])
    
    def applyController(self):
        t = self.ctrl.computePP(self.Quat, self.Omega)
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
        self.CtrlInput = t
    
    def nextStep(self, dt):
        #controller
        if self.UseController:
            self.applyController()
        
        #motor
        self.computeMotor()
        #torque
        self.computeTorque()
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
        self.m1.setConstants(Rho, K_v, K_t, K_tau, I_M, A_swept, A_xsec, Radius, C_D)
        self.m2.setConstants(Rho, K_v, K_t, K_tau, I_M, A_swept, A_xsec, Radius, C_D)
        self.m3.setConstants(Rho, K_v, K_t, K_tau, I_M, A_swept, A_xsec, Radius, C_D)
        self.m4.setConstants(Rho, K_v, K_t, K_tau, I_M, A_swept, A_xsec, Radius, C_D)
    
    def computeMotor(self):
        if not self.TorqueIsSet:
            self.m1.setMeasure(self.CtrlInput[0])
            self.m2.setMeasure(self.CtrlInput[1])
            self.m3.setMeasure(self.CtrlInput[2])
            self.m4.setMeasure(self.CtrlInput[3])
    
    def computeTorque(self):
        if not self.TorqueIsSet:
            self.Torque = Vector([self.L*(self.m1.Thrust-self.m3.Thrust), self.L*(self.m2.Thrust-self.m4.Thrust), self.m1.Tau_z+self.m2.Tau_z+self.m3.Tau_z+self.m4.Tau_z])
        else:
            self.Torque = self.CtrlInput
    
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
        self.ThetaDot = mat1*self.Omega

    def computeAngles(self, dt):
        self.Angles = self.Angles + self.ThetaDot*dt
        
    def computeQuaternion(self, dt):
        self.Quat = Quaternion([self.Angles[0], self.Angles[1], self.Angles[2]])
        
    def mass(self):
        return self.Mass
    
    def setOmega(self, v):
        self.Omega = v
    
    def setTorque(self, t):
        self.Torque = t
    
    def setCtrlInput(self, i):
        self.CtrlInput = i