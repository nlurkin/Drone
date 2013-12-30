#from qmath.qmathcore import quaternion
from ParamsClass import Params
from mathclasses import Matrix, Quaternion, Vector, createRotation

class PSquare:
    '''
    classdocs
    '''
    
    QRef = Quaternion()
    VRef = Vector()
    PQ = None
    POmega = None
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
        
    def computePP(self, qM, omegaM, vM):
        vErr = self.VRef-vM
        
        print "AReq " + (-Params.Gravity)
        print "vErr " + vErr
        #self.Thrust = (vErr*self.PV + self.ARef*self.PA)/self.Mass
        self.Thrust = vErr
        self.Thrust -= Params.Gravity
        
        print "Thrust " + self.Thrust
        if self.Thrust[0]!=0 or self.Thrust[1]!=0:
            qRef = createRotation(self.Thrust, Vector([0,0,1]))
            qRef.w = 1
            qRef.normalize()
        else:
            qRef = self.QRef

        print "qm inv " + qM.conj()
        print "qRef " + qRef
        T = self.Thrust.rotate(qM.conj())*self.Mass
        
        print "T " + T
        T = T[2]

        print "T " + str(T)
        qErr = qRef * qM.conj()

        if qErr[0]<0:
            axisErr = -Vector(qErr)
        else:
            axisErr = Vector(qErr)
        
        self.Torque = self.I*(axisErr*self.PQ - omegaM*self.POmega)
        
        if Params.TorqueIsSet:
            return self.Torque
        deno = ((self.Km1[0])*((self.Km2[1])*((self.Km3[2])*(self.Km4[3])+(self.Km4[2])*(self.Km3[3]))+(self.Km4[1])*((self.Km2[2])*(self.Km3[3])+(self.Km3[2])*(self.Km2[3])))+(self.Km3[0])*((self.Km2[1])*((self.Km1[2])*(self.Km4[3])+(self.Km4[2])*(self.Km1[3]))+(self.Km4[1])*((self.Km1[2])*(self.Km2[3])+(self.Km2[2])*(self.Km1[3]))))
        p1=((self.Km3[0])*((self.Km2[1])*((self.Km4[2])*T+(self.Torque[2])*(self.Km4[3]))+(self.Km4[1])*((self.Km2[2])*T+(self.Torque[2])*(self.Km2[3]))+(self.Torque[1])*((self.Km2[2])*(self.Km4[3])-(self.Km4[2])*(self.Km2[3])))+(self.Torque[0])*((self.Km2[1])*((self.Km3[2])*(self.Km4[3])+(self.Km4[2])*(self.Km3[3]))+(self.Km4[1])*((self.Km2[2])*(self.Km3[3])+(self.Km3[2])*(self.Km2[3]))))/deno
        p2=-((self.Km1[0])*((self.Km4[1])*((self.Torque[2])*(self.Km3[3])-(self.Km3[2])*T)+(self.Torque[1])*(-(self.Km3[2])*(self.Km4[3])-(self.Km4[2])*(self.Km3[3])))+(self.Km3[0])*((self.Km4[1])*((self.Torque[2])*(self.Km1[3])-(self.Km1[2])*T)+(self.Torque[1])*(-(self.Km1[2])*(self.Km4[3])-(self.Km4[2])*(self.Km1[3])))+(self.Torque[0])*(self.Km4[1])*((self.Km3[2])*(self.Km1[3])-(self.Km1[2])*(self.Km3[3])))/deno
        p3=-((self.Km1[0])*((self.Km2[1])*(-(self.Km4[2])*T-(self.Torque[2])*(self.Km4[3]))+(self.Km4[1])*(-(self.Km2[2])*T-(self.Torque[2])*(self.Km2[3]))+(self.Torque[1])*((self.Km4[2])*(self.Km2[3])-(self.Km2[2])*(self.Km4[3])))+(self.Torque[0])*((self.Km2[1])*((self.Km1[2])*(self.Km4[3])+(self.Km4[2])*(self.Km1[3]))+(self.Km4[1])*((self.Km1[2])*(self.Km2[3])+(self.Km2[2])*(self.Km1[3]))))/deno
        p4=-((self.Km1[0])*((self.Km2[1])*((self.Torque[2])*(self.Km3[3])-(self.Km3[2])*T)+(self.Torque[1])*((self.Km2[2])*(self.Km3[3])+(self.Km3[2])*(self.Km2[3])))+(self.Km3[0])*((self.Km2[1])*((self.Torque[2])*(self.Km1[3])-(self.Km1[2])*T)+(self.Torque[1])*((self.Km1[2])*(self.Km2[3])+(self.Km2[2])*(self.Km1[3])))+(self.Torque[0])*(self.Km2[1])*((self.Km3[2])*(self.Km1[3])-(self.Km1[2])*(self.Km3[3])))/deno
        self.Output = Vector([p1,p2,p3,p4])
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