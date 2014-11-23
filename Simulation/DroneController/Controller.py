#from qmath.qmathcore import quaternion
import pickle

from numpy.lib.scimath import sqrt

from DroneController.Calibrator import Calibrator
from DroneMath.mathclasses import Matrix, Quaternion, Vector, createRotation
from ParamsClass import Params


class SensorValues:
    
    def __init__(self):
        self.Acceleration = Vector()
        self.Omega = Vector()
        self.Quat = Quaternion()
        self.Alpha = Vector()
    
    def setValues(self, acceleration, gyroscope, quat, alpha):
        #self.Acceleration = acceleration
        #self.Omega = gyroscope
        #self.Quat = quat
        #self.Alpha = alpha
        #return
        oldGyro = Vector(self.Omega)
        self.OmegaVector.append(oldGyro)
        self.Acceleration[0] = int(acceleration[0]*8192*2)/(8192.*2.)
        self.Acceleration[1] = int(acceleration[1]*8192*2)/(8192.*2.)
        self.Acceleration[2] = int(acceleration[2]*8192*2)/(8192.*2.)
        
        self.Omega[0] = int(gyroscope[0]*131*2)/(131.*2.)
        self.Omega[1] = int(gyroscope[1]*131*2)/(131.*2.)
        self.Omega[2] = int(gyroscope[2]*131*2)/(131.*2.)
        
        self.Quat = quat
        
        self.computeAlpha(oldGyro)
    
    def computeAlpha(self, oldGyroscope):
        dt = Params.dt
        self.Alpha = (self.Omega - oldGyroscope)/dt;


class PSquare:
    '''
    classdocs
    '''
    lastqRef = Quaternion()
    lastvRef = Vector()
    
    QRef = Quaternion()
    VRef = Vector()
    PQ = None
    POmega = None
    PInt = None
    I = Matrix()
    Torque = Vector()
    Thrust = Vector()
    
    Km1 = None
    Km2 = None
    Km3 = None
    Km4 = None
    
    Output = None

    K = None
    b = None
    L = None
    Mass = None
    
    QInt = Quaternion()
    

    def __init__(self):
        '''
        Constructor
        '''
        #self.QRef = qmath.quaternion([0, 0, 0])
        self.QRef = Quaternion([0,0,0])
        self.POmega = 0
        self.PQ = 0
        self.PV = 0
        self.PA = 0
        self.I = Vector([0, 0, 0])
        self.Torque = Vector([0, 0, 0])
        self.Output = Vector([0,0,0,0])
        self.VRef = Vector([0,0,0])
        self.Thrust = Vector([0,0,0])
    
    def setI(self, I):
        self.I = Matrix([[I[0],0,0],[0,I[1],0],[0,0,I[2]]])
    
    def setPs(self, v):
        self.PQ = v[0]
        self.POmega = v[1]
        self.PInt = v[2]
        
    def setQRef(self, q, v):
        self.QRef = q
        self.VRef = v
    
    def setMotorCoefficient(self, m1, m2, m3, m4):
        self.Km1 = m1
        self.Km2 = m2
        self.Km3 = m3
        self.Km4 = m4
    
    def setParameters(self, k,L,b,m):
        self.K = k
        self.L = L
        self.b = b
        self.Mass = m
    
    def controlSignal(self):
        if self.QRef.vector().mag()==0:
            aControl = 0
        else:
            aControl = 1

        if self.VRef.mag()==0 and aControl!=0:    
            vControl = 0
        else:
            vControl = 1
        
        return [vControl, aControl]
    
    def velocityControl(self, qM, aM, vM, vControl):
        if vControl:
            vErr = self.VRef-vM
            self.lastvRef = self.VRef
        else:
            vErr = Vector([aM[0], aM[1], 0])
            self.lastvRef = Vector([0,0,0])
        
        #TODO: try having a real PID with here rather than P with Kp=1
        
        #print "AReq " + (-Params.Gravity)
        #print "vErr " + vErr
        
        #TODO: try multiplying by the mass here as this is actually computing the acceleration
        Thrust = vErr - Params.Gravity
        
        qRef = createRotation(Thrust, Vector([0,0,1]))
        #This is required to keep the z component compensating for gravity. 
        #If not, the norm is kept when rotating and the z component is reduced accordingly.
        #Maybe try by creating the rotation between z and the error and adding the gravity after!
        #TODO: ou alors c'est fait plus loin en divisant par z' et celui-ci ne sert a rien?
        qRef.w = 1
        qRef.normalize()
        
        return [Thrust, qRef]
    
    def attitudeControl(self, qM, omegaM, qRef, dt):
        qErr = qRef * qM.conj()
        
        self.QInt = self.QInt + qErr*dt*self.PInt
        #TODO: QInt here?
        if qErr[0]<0:
            intErr = -Vector(self.QInt)
        else:
            intErr = Vector(self.QInt)

        if qErr[0]<0:
            axisErr = -Vector(qErr)
        else:
            axisErr = Vector(qErr)
        
        
        #PInt>>0 -> precision; PInt<<0 -> Reaction speed
        Torque = self.I*(axisErr*self.PQ - omegaM*self.POmega - intErr)
        #Iinv = Matrix([[self.I[0][0],0,0],[0,self.I[1][1],0],[0,0,self.I[2][2]]])
        #Torque = Iinv*(axisErr*self.PQ - omegaM*self.POmega)
        return Torque
        
    def computePP(self, qM, omegaM, aM, vM, dt):
        #print "qM " + qM
        #print "omegaM " + omegaM
        #print "aM " + aM
        #print "vM " + vM
        
        print "VRef " + self.VRef
        print "QRef " + self.QRef
        
        [vControl, aControl] = self.controlSignal()
        
        print "Control signal : " + str(vControl) + " " + str(aControl) 
        
        [self.Thrust, vQRef] = self.velocityControl(qM, aM, vM, vControl)
        
        #print "qm inv " + qM.conj()
        #print "Thrust " + self.Thrust*self.Mass + " (" + str(sqrt(self.Thrust.mag())) + ")"
        #print "Rotation " + qM
        
        #TODO: remove the -qM as -qM==qM
        #TODO: je pense qu'en ce qui concerne l'axe z ici, comme on rotate z et qu'on prends la composante z, q ou q* donne les meme resultat
        #TODO: je pense que c'est ca qui rescale T pour contrer correctement la gravite. 
        T = self.Thrust*self.Mass*(1/(Vector([0,0,1]).rotate(-qM.conj())[2]))
        #T = self.Thrust.rotate(-qM.conj())*self.Mass
        
        #print "T " + T + " (" + str(sqrt(T.mag())) + ")"
        T = T[2]
        #print "T " + str(T)
        
        if vControl or not aControl:
            qRef = vQRef
        else:
            qRef = self.QRef
        
        print "qRef for torque " + qRef
        self.lastqRef = qRef
        self.Torque = self.attitudeControl(qM, omegaM, qRef, dt)
        #fit
        if Params.TorqueIsSet:
            return self.Torque
        
        #print "Requested T " + str(T)
        #print "Requested Torque " + self.Torque
        #print self.Km1
        #print self.Km2
        #print self.Km3
        #print self.Km4
        deno = (self.Km1[0]*(self.Km2[1]*(self.Km3[2]*self.Km4[3]+self.Km4[2]*self.Km3[3])+self.Km4[1]*(self.Km2[2]*self.Km3[3]+self.Km3[2]*self.Km2[3]))+self.Km3[0]*(self.Km2[1]*(self.Km1[2]*self.Km4[3]+self.Km4[2]*self.Km1[3])+self.Km4[1]*(self.Km1[2]*self.Km2[3]+self.Km2[2]*self.Km1[3])))
        #print deno
        p1=((self.Km3[0])*((self.Km2[1])*((self.Km4[2])*T+(self.Torque[2])*(self.Km4[3]))+(self.Km4[1])*((self.Km2[2])*T+(self.Torque[2])*(self.Km2[3]))+(self.Torque[1])*((self.Km2[2])*(self.Km4[3])-(self.Km4[2])*(self.Km2[3])))+(self.Torque[0])*((self.Km2[1])*((self.Km3[2])*(self.Km4[3])+(self.Km4[2])*(self.Km3[3]))+(self.Km4[1])*((self.Km2[2])*(self.Km3[3])+(self.Km3[2])*(self.Km2[3]))))/deno
        p2=-((self.Km1[0])*((self.Km4[1])*((self.Torque[2])*(self.Km3[3])-(self.Km3[2])*T)+(self.Torque[1])*(-(self.Km3[2])*(self.Km4[3])-(self.Km4[2])*(self.Km3[3])))+(self.Km3[0])*((self.Km4[1])*((self.Torque[2])*(self.Km1[3])-(self.Km1[2])*T)+(self.Torque[1])*(-(self.Km1[2])*(self.Km4[3])-(self.Km4[2])*(self.Km1[3])))+(self.Torque[0])*(self.Km4[1])*((self.Km3[2])*(self.Km1[3])-(self.Km1[2])*(self.Km3[3])))/deno
        p3=-((self.Km1[0])*((self.Km2[1])*(-(self.Km4[2])*T-(self.Torque[2])*(self.Km4[3]))+(self.Km4[1])*(-(self.Km2[2])*T-(self.Torque[2])*(self.Km2[3]))+(self.Torque[1])*((self.Km4[2])*(self.Km2[3])-(self.Km2[2])*(self.Km4[3])))+(self.Torque[0])*((self.Km2[1])*((self.Km1[2])*(self.Km4[3])+(self.Km4[2])*(self.Km1[3]))+(self.Km4[1])*((self.Km1[2])*(self.Km2[3])+(self.Km2[2])*(self.Km1[3]))))/deno
        p4=-((self.Km1[0])*((self.Km2[1])*((self.Torque[2])*(self.Km3[3])-(self.Km3[2])*T)+(self.Torque[1])*((self.Km2[2])*(self.Km3[3])+(self.Km3[2])*(self.Km2[3])))+(self.Km3[0])*((self.Km2[1])*((self.Torque[2])*(self.Km1[3])-(self.Km1[2])*T)+(self.Torque[1])*((self.Km1[2])*(self.Km2[3])+(self.Km2[2])*(self.Km1[3])))+(self.Torque[0])*(self.Km2[1])*((self.Km3[2])*(self.Km1[3])-(self.Km1[2])*(self.Km3[3])))/deno
        self.Output = Vector([p1,p2,p3,p4])
        #print "Torque0 = " + str(p1*self.Km1[0]-p3*self.Km3[0])
        #print "Torque1 = " + str(p2*self.Km2[1]-p4*self.Km4[1])
        #print "Torque2 = " + str(p1*self.Km1[2]-p2*self.Km2[2]+p3*self.Km3[2]-p4*self.Km4[2])
        #print "T = " + str(p1*self.Km1[3]+p2*self.Km2[3]+p3*self.Km3[3]+p4*self.Km4[3])
        return self.Output


class PID:
    '''
    classdocs
    '''
    
    Kd = None
    Kp = None
    Ki = None
    
    QRef = None
    ARef = None
    I = Matrix()
    Torque = Vector()


    def __init__(self):
        '''
        Constructor
        '''
        #self.QRef = qmath.quaternion([0, 0, 0])
        self.Kd = 0
        self.Kp = 0
        self.ki = 0
        self.QRef = Quaternion([0,0,0])
        self.ARef = Vector([0,0,0])
        self.I = Vector([0, 0, 0])
        self.Torque = Vector([0, 0, 0])
    
    def setI(self, I):
        self.I = Matrix([[I[0],0,0],[0,I[1],0],[0,0,I[2]]])
    
    def setPs(self, v):
        self.Kd = v[0]
        self.Kp = v[1]
        self.Ki = v[2]
        
    def setQRef(self, ref):
        self.QRef = ref
    
    def computePP(self, qM, omegaM):
        qErr = -Vector(self.QRef * qM.conj())
        
        self.Torque = self.I * (-self.Kd*omegaM - self.Kp*qErr)
        return self.Torque
    
class Calibration(object):
    
    def __init__(self, body, simu):
        self.cali = Calibrator()
        self.body = body
        self.simu = simu
    
    def calibrateI(self, dt, minMotor):
        dMotor = 1
        self.body.CtrlInput = [minMotor, minMotor, minMotor, minMotor]
        self.body.CtrlInput[0] = minMotor+dMotor
    
        for j in range(0, int(0.1/dt)):
            self.simu.nextStep()
        
        for i in range(0,2):
            #set motor power
            self.body.CtrlInput[0] = minMotor + pow(-1,i)*dMotor
            #get point
            for j in range(0, int(0.1/dt)):
                self.simu.nextStep()
            
            newOmega = Vector(self.body.sensors.Omega)
            newAlpha = Vector(self.body.sensors.Alpha)
            #newOmega = self.body.Omega
            #newAlpha = self.body.Alpha
            #newOmega[0] = simu.test.omega
            #newAlpha[0] = simu.test.alpha
            #self.cali.newPoint(self.CtrlInput[0], self.Omega, self.Alpha, 0, 0)
            print newOmega
            self.cali.newPoint(self.body.CtrlInput[0], newOmega, newAlpha, 0, 0)
            for j in range(0, int(0.1/dt)):
                self.simu.nextStep()
            newOmega = Vector(self.body.sensors.Omega)
            newAlpha = Vector(self.body.sensors.Alpha)
            #newOmega = self.body.Omega
            #newAlpha = self.body.Alpha
            #newOmega[0] = simu.test.omega
            #newAlpha[0] = simu.test.alpha
            
            print newOmega
            self.cali.newPoint(self.body.CtrlInput[0], newOmega, newAlpha, 0, 0)
            self.cali.calibrateI()
            self.cali.clearPoints()
        
        self.body.CtrlInput[0] = minMotor-dMotor
        for j in range(0, int(0.1/dt)):
            self.simu.nextStep()
        
        self.body.CtrlInput =  [0,0,0,0]

    def calibrateMotor(self, motor, dt, minMotor):
        dMotor = 3
        self.body.CtrlInput = [minMotor, minMotor, minMotor, minMotor]

        self.body.CtrlInput[motor] = minMotor+dMotor
        for j in range(0, int(0.1/dt)):
            self.simu.nextStep()
        #get point
        for j in range(0, int(0.1/dt)):
            self.simu.nextStep()
            
        self.cali.newPoint(self.body.CtrlInput[motor]-minMotor, self.body.sensors.Omega, self.body.sensors.Alpha, self.body.sensors.Acceleration, self.body.sensors.Quat)

        self.body.CtrlInput[motor] = minMotor+2*dMotor
        for j in range(0, int(0.1/dt)):
            self.simu.nextStep()
        #get point
        for j in range(0, int(0.1/dt)):
            self.simu.nextStep()
        self.cali.newPoint(self.body.CtrlInput[motor]-minMotor, self.body.sensors.Omega, self.body.sensors.Alpha, self.body.sensors.Acceleration, self.body.sensors.Quat)

        self.cali.calibrateR(motor,Params.Mass)#self.m1.K*pow(Params.MaxOmega,2)/(10000))
        self.cali.clearPoints()
        
        self.body.CtrlInput[motor] = minMotor-2*dMotor
        for j in range(0, int(0.2/dt)):
            self.simu.nextStep()
        self.body.CtrlInput[motor] = minMotor-dMotor
        for j in range(0, int(0.2/dt)):
            self.simu.nextStep()
        
        self.body.CtrlInput =  [0,0,0,0]
    
    def level(self):
        minMotor = 0
        minMotor+=1
        self.body.CtrlInput = [minMotor, minMotor, minMotor, minMotor]
        self.simu.nextStep()
        while(self.Acceleration[2]<0):
            minMotor+=1
            self.body.CtrlInput = [minMotor, minMotor, minMotor, minMotor]
            self.simu.nextStep() 
        
        return minMotor
        
    def exportCalib(self):
        exportVals = {"R": [self.cali.getR(0),self.cali.getR(1),self.cali.getR(2),self.cali.getR(3)], "I":self.cali.getIAxis()}
        oFile = open('calib.dat', "wb")
        pickle.dump(exportVals, oFile)
        oFile.close()
    
    def importCalib(self):
        oFile = open('calib.dat', "rb")
        vals = pickle.load(oFile)
        oFile.close()
        R = vals["R"]
        I = vals["I"]
        self.body.ctrl.setMotorCoefficient(R[0],R[1],R[2],R[3])
        self.body.ctrl.setI(I)
    