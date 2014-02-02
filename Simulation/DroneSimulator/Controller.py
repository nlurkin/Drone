#from qmath.qmathcore import quaternion
from ParamsClass import Params
from mathclasses import Matrix, Quaternion, Vector, createRotation
from numpy.lib.scimath import sqrt

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
        
        #print "AReq " + (-Params.Gravity)
        #print "vErr " + vErr
        Thrust = vErr - Params.Gravity
        
        qRef = createRotation(Thrust, Vector([0,0,1]))
        qRef.w = 1
        qRef.normalize()
        
        return [Thrust, qRef]
    
    def attitudeControl(self, qM, omegaM, qRef, dt):
        qErr = qRef * qM.conj()
        
        self.QInt = self.QInt + qErr*dt
        if qErr[0]<0:
            intErr = -Vector(self.QInt)
        else:
            intErr = Vector(self.QInt)

        if qErr[0]<0:
            axisErr = -Vector(qErr)
        else:
            axisErr = Vector(qErr)
        
        
        #PInt>>0 -> precision; PInt<<0 -> Reaction speed
        Torque = self.I*(axisErr*self.PQ - omegaM*self.POmega - intErr*self.PInt)
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