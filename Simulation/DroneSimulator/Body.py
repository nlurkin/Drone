import sys

from numpy.ma.core import sin, tan, cos
from scipy.constants.constants import pi

from Calibrator import Calibrator
from Controller import PID, PSquare
from Motor import Motor
from ParamsClass import Params
from mathclasses import Matrix, Quaternion, Vector


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
    
    cali = Calibrator()
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
        print "inertia factor x " + str((self.I[1]-self.I[2])/self.I[0])
        
    def setParameters(self,TorqueIsSet, MaxTorque, UseController):
        self.TorqueIsSet = TorqueIsSet
        self.MaxTorque = MaxTorque 
        self.UseController = UseController
    
    def setReference(self, qRef, vRef):
        self.ctrl.setQRef(qRef, vRef)
    
    def initController(self):
        self.ctrl.setI(self.I)
        if Params.ctrlType==0:
            self.ctrl.setPs([10,15,2])
        elif Params.ctrlType==1:
            self.ctrl.setPs([20,4])
    
    def applyController(self):
        t = self.ctrl.computePP(self.Quat, self.Omega, self.Acceleration, self.Velocity)
        if self.TorqueIsSet: 
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
        
        oldomega = Vector(self.Omega)
        #motor
        self.computeMotor(dt)
        #torque
        self.computeTorque()
        #thrust
        self.computeThrust()
        #fiction
        self.computeFriction()   
        #acceleration
        self.computeAcceleration()
        #vitesse
        self.computeVelocity(dt)
        #position
        self.computePosition(dt)
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
        
        #print "oldomega " + oldomega
        I = (self.I[2]-self.I[1])/self.I[0]
        rx = self.L*self.m1.K*pow(Params.MaxOmega,2)/(10000.*self.I[0])
        '''print "true I " + str(I)
        print "true rx " + str(rx)
        print "true alpha " + str(self.Alpha)
        print "omega motor " + str(self.m1.Omega)
        print "omega ici " + str(self.m1.Power*Params.MaxOmega/100.)
        print "torque " + str(self.Torque[0]/self.I[0])
        print "torque ici " + str(rx*pow(self.m1.Power,2))
        print "alpha second terme " + str(oldomega[1]*oldomega[2]*I)'''
        alpha = pow(self.m1.Power,2)*rx - oldomega[1]*oldomega[2]*I
        Rx = (self.Alpha[0]+oldomega[1]*oldomega[2]*I)/pow(self.m1.Power,2)
        #print "calxulated alpha " + str(alpha)
        print "Calculated rx " + str(Rx)
        
        
    def setMotorConstants(self, Rho, K_v, K_t, K_tau, I_M, A_swept, A_xsec, Radius, C_D):
        self.m1.setConstants(Rho, K_v, K_t, K_tau, I_M, A_swept, A_xsec, Radius, C_D)
        self.m2.setConstants(Rho, K_v, K_t, K_tau, I_M, A_swept, A_xsec, Radius, C_D)
        self.m3.setConstants(Rho, K_v, K_t, K_tau, I_M, A_swept, A_xsec, Radius, C_D)
        self.m4.setConstants(Rho, K_v, K_t, K_tau, I_M, A_swept, A_xsec, Radius, C_D)
        if self.UseController:
            self.ctrl.setMotorCoefficient([self.L*self.m1.K, 0, self.m1.B, self.m1.K],
                                          [0, self.L*self.m2.K, self.m2.B, self.m2.K],
                                          [self.L*self.m3.K, 0, self.m3.B, self.m3.K],
                                          [0, self.L*self.m4.K, self.m4.B, self.m4.K])
            self.ctrl.setParameters(self.m1.K, self.L, self.m1.B, self.Mass)
    
    def computeMotor(self, dt):
        if not self.TorqueIsSet:
            print self.CtrlInput
            self.m1.setMeasure(self.CtrlInput[0], dt)
            self.m2.setMeasure(self.CtrlInput[1], dt)
            self.m3.setMeasure(self.CtrlInput[2], dt)
            self.m4.setMeasure(self.CtrlInput[3], dt)
    
    def computeTorque(self):
        if not self.TorqueIsSet:
            self.Torque = Vector([self.L*(self.m1.Thrust-self.m3.Thrust), self.L*(self.m2.Thrust-self.m4.Thrust), self.m1.Tau_z+self.m2.Tau_z+self.m3.Tau_z+self.m4.Tau_z])
            print "M1 thrust " + str(self.L*self.m1.Thrust)
            print "M2 thrust " + str(self.L*self.m2.Thrust)
            print "M3 thrust " + str(self.L*self.m3.Thrust)
            print "M4 thrust " + str(self.L*self.m4.Thrust)
            print "M1 tauZ " + str(self.m1.Tau_z)
            print "M2 tauZ " + str(self.m2.Tau_z)
            print "M3 tauZ " + str(self.m3.Tau_z)
            print "M4 tauZ " + str(self.m4.Tau_z)
            print "Body torque " + self.Torque
        else:
            self.Torque = self.CtrlInput
        
    def computeFriction(self):        
        self.Friction = -self.Velocity*self.K_d
    
    def computeAlpha(self, dt):
        #print "Omega alpha: " + self.Omega
        self.Alpha = Vector([(self.Torque[0] - (self.I[1]-self.I[2])*self.Omega[1]*self.Omega[2])/self.I[0],
                      (self.Torque[1] - (self.I[2]-self.I[0])*self.Omega[0]*self.Omega[2])/self.I[1],
                      (self.Torque[2] - (self.I[0]-self.I[1])*self.Omega[0]*self.Omega[1])/self.I[2]])
        #print self.Alpha
        #print "alpha second term(true) " + str((self.I[2]-self.I[1])*self.Omega[1]*self.Omega[2]/self.I[0])
    
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
    
    def computeThrust(self):
        if not self.TorqueIsSet:
            self.Thrust = Vector([0, 0, self.m1.Thrust+self.m2.Thrust+self.m3.Thrust+self.m4.Thrust])
        else:
            self.Thrust = Vector([0,0,0])
    
    def computeAcceleration(self):
        #rotate thrust
        print "Thrust before rotation " + self.Thrust
        print "Rotation " + self.Quat
        T = self.Thrust.rotate(self.Quat)
        print "Thrust " + T
        self.Acceleration = T/self.Mass
        self.Acceleration = Params.Gravity+T*(1/self.Mass)+self.Friction
    
    def computeVelocity(self, dt):
        self.Velocity += self.Acceleration*dt
    
    def computePosition(self, dt):
        self.Position += self.Velocity*dt
    
    def calibrate(self,dt):
        self.UseController = False
        
        for i in range(0,2):
            #set motor power
            #self.m1.setMeasure(40+i*5, dt)
            self.CtrlInput = [3+i*2, 0, 0, 0]
            #get point
            self.nextStep(dt)
            self.cali.estimateI(0, self.Alpha, self.Omega)
            self.nextStep(dt)
            self.cali.estimateI(0, self.Alpha, self.Omega)
            self.nextStep(dt)
            #self.cali.newOmega(0,self.Omega)
            self.cali.estimateI(0, self.Alpha, self.Omega)
            self.nextStep(dt)
            self.cali.estimateI(0, self.Alpha, self.Omega)
            self.nextStep(dt)
            self.cali.estimateI(0, self.Alpha, self.Omega)
            self.nextStep(dt)
            self.cali.estimateI(0, self.Alpha, self.Omega)
            self.nextStep(dt)
            #self.cali.newOmega(0,self.Omega)
            self.nextStep(dt)
            #set point
            self.cali.newPoint(0, 3+i*2, self.Omega, self.Alpha)
        
        
        print self.cali.fI
        
        self.UseController = False
        

if __name__ == "__main__":
    K_d = 0.0013
    I = Params.I
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
    b = Body()
    b.setModel(I, K_d, Params.L, Params.Mass)
    b.setParameters(Params.TorqueIsSet, Params.MaxTorque, True)
    b.setMotorConstants(Rho, K_v, K_t, K_tau, I_M, A_swept, A_xsec, Radius, C_D)
    b.calibrate(0.0001)
    sys.exit(0);
