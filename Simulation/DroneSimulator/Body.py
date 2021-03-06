import sys

from numpy.ma.core import sin, tan, cos
from scipy.constants.constants import pi

from DroneController.Calibrator import Calibrator
from DroneController.Controller import PID, PSquare, Calibration, SensorValues
from DroneMath.mathclasses import Matrix, Quaternion, Vector
from Motor import Motor
from ParamsClass import Params


class Body(object):
    '''
    classdocs
    '''

    oldomega = Vector()
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
    CtrlInput = [0,0,0,0]
    
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
    
    def __init__(self, simu):
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
        
        self.sensors = SensorValues()
        self.calib = Calibration(self, simu)
    
    def setModel(self, I, K_d, L, Mass):
        self.I = I
        self.K_d = K_d
        self.L = L
        self.Mass = Mass
        #print "inertia factor x " + str((self.I[1]-self.I[2])/self.I[0])
        
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
            self.ctrl.setPs(Params.PCoeff)
    
    def applyController(self):
        t = self.ctrl.computePP(self.Quat, self.Omega, self.Acceleration, self.Velocity, Params.dt)
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
        #print "Controller input " + t
    
    def changeInput(self, motor, value):
        self.CtrlInput[motor] = value;
        print "CtrlInput is now " + str(self.CtrlInput)
        
    def nextStep(self, dt):
        #controller
        if self.UseController:
            self.applyController()
        
        ooldomega = Vector(self.Omega)
        oldaplha = Vector(self.Alpha)
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
        
        self.sensors.setValues(self.Acceleration, self.Omega, self.Quat, self.Alpha)
        
        ''''I = (oldaplha[0]-self.Alpha[0])/(ooldomega[1]*ooldomega[2]-self.Omega[1]*self.Omega[2])
        if I > 1:
            #print "oldomega " + oldomega
            print "oldomega " + ooldomega
            print "omega " + self.Omega
            print "oldalpha " + oldaplha
            print "alpha " + self.Alpha
            #print "self.oldomega + " + self.oldomega
            print "diff " + str(ooldomega[1]*ooldomega[2]-self.Omega[1]*self.Omega[2]) 
            print "I Body " + str(I)'''
        #I = (self.I[2]-self.I[1])/self.I[0]
        #rx = self.L*self.m1.K*pow(Params.MaxOmega,2)/(10000.*self.I[0])
        '''print "true I " + str(I)
        print "true rx " + str(rx)
        print "true alpha " + str(self.Alpha)
        print "omega motor " + str(self.m1.Omega)
        print "omega ici " + str(self.m1.Power*Params.MaxOmega/100.)
        print "torque " + str(self.Torque[0]/self.I[0])
        print "torque ici " + str(rx*pow(self.m1.Power,2))
        print "alpha second terme " + str(oldomega[1]*oldomega[2]*I)'''
        #alpha = pow(self.m1.Power,2)*rx - ooldomega[1]*ooldomega[2]*I
        #I = -0.887005649718
        #Rx = (self.Alpha[0]+ooldomega[1]*ooldomega[2]*I)/pow(self.m1.Power,2)
        #print Rx
        #print "calxulated alpha " + str(alpha)
        #print "Calculated rx " + str(Rx)
        #self.oldomega = Vector(ooldomega)
        
        
    def setMotorConstants(self, Rho, K_v, K_t, K_tau, I_M, A_swept, A_xsec, Radius, C_D):
        self.m1.setConstants(Rho, K_v, K_t, K_tau, I_M, A_swept, A_xsec, Radius, C_D)
        self.m2.setConstants(Rho, K_v, K_t, K_tau, I_M, A_swept, A_xsec, Radius, C_D)
        self.m3.setConstants(Rho, K_v, K_t, K_tau, I_M, A_swept, A_xsec, Radius, C_D)
        self.m4.setConstants(Rho, K_v, K_t, K_tau, I_M, A_swept, A_xsec, Radius, C_D)
        if self.UseController:
            Rx = self.L*self.m1.K*pow(Params.MaxOmega,2)/(10000)
            Ry = self.L*self.m1.K*pow(Params.MaxOmega,2)/(10000)
            Rz = self.m1.B*pow(Params.MaxOmega,2)/(10000)
            RT = self.m1.K*pow(Params.MaxOmega,2)/(10000)
            print "Real parameters :"
            print "R1 " + str([Rx, 0, Rz, RT])
            print "R2 " + str([0, Ry, Rz, RT])
            print "R3 " + str([Rx, 0, Rz, RT])
            print "R4 " + str([0, Ry, Rz, RT])
            print Params.I
            print "I_1 " + str((Params.I[1]-Params.I[2])/Params.I[0])
            print "I_2 " + str((Params.I[0]-Params.I[2])/Params.I[1])
            print "I_3 " + str((Params.I[0]-Params.I[1])/Params.I[2])
            self.ctrl.setMotorCoefficient([Rx, 0, Rz, RT],
                                          [0, Ry, Rz, RT],
                                          [Rx, 0, Rz, RT],
                                          [0, Ry, Rz, RT])
            self.ctrl.setParameters(self.m1.K, self.L, self.m1.B, self.Mass)
    
    def computeMotor(self, dt):
        if not self.TorqueIsSet:
            self.m1.setMeasure(self.CtrlInput[0], dt)
            self.m2.setMeasure(self.CtrlInput[1], dt)
            self.m3.setMeasure(self.CtrlInput[2], dt)
            self.m4.setMeasure(self.CtrlInput[3], dt)
    
    def computeTorque(self):
        if not self.TorqueIsSet:
            self.Torque = Vector([self.L*(self.m1.Thrust-self.m3.Thrust), self.L*(self.m2.Thrust-self.m4.Thrust), self.m1.Tau_z+self.m2.Tau_z+self.m3.Tau_z+self.m4.Tau_z])
            '''print "M1 thrust " + str(self.L*self.m1.Thrust)
            print "M2 thrust " + str(self.L*self.m2.Thrust)
            print "M3 thrust " + str(self.L*self.m3.Thrust)
            print "M4 thrust " + str(self.L*self.m4.Thrust)
            print "M1 tauZ " + str(self.m1.Tau_z)
            print "M2 tauZ " + str(self.m2.Tau_z)
            print "M3 tauZ " + str(self.m3.Tau_z)
            print "M4 tauZ " + str(self.m4.Tau_z)'''
            #print "Body torque " + self.Torque
        else:
            self.Torque = self.CtrlInput
        
    def computeFriction(self):        
        self.Friction = -self.Velocity*self.K_d
    
    def computeAlpha(self, dt):
        #print "Omega alpha: " + self.Omega
        #oldalpha = self.Alpha
        self.Alpha = Vector([(self.Torque[0] - (self.I[1]-self.I[2])*self.Omega[1]*self.Omega[2])/self.I[0],
                      (self.Torque[1] - (self.I[2]-self.I[0])*self.Omega[0]*self.Omega[2])/self.I[1],
                      (self.Torque[2] - (self.I[0]-self.I[1])*self.Omega[0]*self.Omega[1])/self.I[2]])
        #print "I factor apha " + str((self.Alpha[0] - oldalpha[0])/(self.Omega[1]*self.Omega[2] - self.oldomega[1]*self.oldomega[2]))
        #print self.Alpha
        #print "alpha second term(true) " + str((self.I[2]-self.I[1])*self.Omega[1]*self.Omega[2]/self.I[0])
    
    def computeOmega(self, dt):
        self.Omega = self.Omega + self.Alpha*dt
        #print self.Omega
    
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
        #print "Thrust before rotation " + self.Thrust
        #print "Rotation " + self.Quat
        T = self.Thrust.rotate(self.Quat)
        #print "Thrust " + T
        self.Acceleration = T/self.Mass
        self.Acceleration = Params.Gravity+T*(1/self.Mass)+self.Friction*(1/self.Mass)
    
    def computeVelocity(self, dt):
        #print dt
        self.Velocity += self.Acceleration*dt
        #print self.Velocity
    
    def computePosition(self, dt):
        self.Position += self.Velocity*dt
        if(Params.ground and self.Position[2]<0):
            #On the ground. Reset all values
            self.Position[2]=0
            self.Velocity = Vector([0,0,0])
            self.Acceleration = Vector([0,0,0])
            self.Alpha = Vector([0,0,0])
            self.Omega = Vector([0,0,0])
            self.ThetaDot = Vector([0,0,0])
            self.Angles = Vector([0,0,0])
            self.Quat = Quaternion([self.Angles[0], self.Angles[1], self.Angles[2]])
    
    def calibrate(self,dt):
        #dt = 0.001
        self.UseController = False
        
        #minMotor = self.level(simu)
        #print minMotor
        minMotor = 886
        
        self.calib.calibrateI(dt, minMotor)
        self.calib.calibrateMotor(0, dt, minMotor)
        self.calib.calibrateMotor(1, dt, minMotor)
        self.calib.calibrateMotor(2, dt, minMotor)
        self.calib.calibrateMotor(3, dt, minMotor)
        print self.calib.cali.getAveragedI()
        print self.calib.cali.getIAxis()
        print self.calib.cali.getR(0)
        print self.calib.cali.getR(1)
        print self.calib.cali.getR(2)
        print self.calib.cali.getR(3)
        self.UseController = True
        self.ctrl.setMotorCoefficient( self.calib.cali.getR(0), self.calib.cali.getR(1), self.calib.cali.getR(2), self.calib.cali.getR(3))
        self.ctrl.setI( self.calib.cali.getIAxis())
        
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
    b.calibrate(0.001)
    sys.exit(0);
