#from qmath.qmathcore import quaternion
from mathclasses import Matrix
from mathclasses import Quaternion, Vector

class PSquare:
    '''
    classdocs
    '''
    
    QRef = Quaternion()
    PQ = None
    POmega = None
    I = Matrix()
    Torque = Vector()


    def __init__(self):
        '''
        Constructor
        '''
        #self.QRef = qmath.quaternion([0, 0, 0])
        self.QRef = Quaternion([0,0,0])
        self.POmega = 0
        self.PQ = 0
        self.I = Vector([0, 0, 0])
        self.Torque = Vector([0, 0, 0])
    
    def setI(self, I):
        self.I = Matrix([[I[0],0,0],[0,I[1],0],[0,0,I[2]]])
    
    def setPs(self, v):
        self.PQ = v[0]
        self.POmega = v[1]
        
    def setQRef(self, q):
        self.QRef = q
    
    def computePP(self, qM, omegaM):
        qErr = self.QRef * qM.conj()
        if qErr[0]<0:
            axisErr = -Vector(qErr)
        else:
            axisErr = Vector(qErr)
        
        self.Torque = self.I*(axisErr*self.PQ - omegaM*self.POmega)
        
        return self.Torque


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