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
    I = Vector()
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
        self.I = I
    
    def setPs(self, v):
        self.PQ = v[0]
        self.POmega = v[1]
        
    def setQRef(self, q):
        self.QRef = q
    
    def computePP(self, qM, omegaM):
        print "QRef " + self.QRef
        print "qM " + qM
        
        qErr = self.QRef.conj() * qM
        print "qErr " + qErr
        if qErr[0]<0:
            axisErr = -Vector(qErr)
        else:
            axisErr = Vector(qErr)
        
        print "Err " + axisErr
        self.Torque = -axisErr*self.PQ - omegaM*self.POmega
        #self.Torque[0] = -axisErr[0]*self.PQ - omegaM[0]*self.POmega
        #self.Torque[1] = -axisErr[1]*self.PQ - omegaM[1]*self.POmega
        #self.Torque[2] = -axisErr[2]*self.PQ - omegaM[2]*self.POmega
        
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
    I = Vector()
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
        
    def setQRef(self, ref, angle):
        self.QRef = ref
        self.ARef = angle
    
    def computePP(self, qM, omegaM, angles):
        #print "QRef " + self.QRef
        #print "QM " + qM
        #qErr = self.QRef * qM.conj()
        
        #print "qErr " + qErr
        #print qErr.mag()
        #aRef = self.QRef.angles()
        print "ARef " + self.ARef
        print "AM " + angles 
        aErr = -(self.ARef-angles)
        print "aErr " + aErr
        self.Torque = self.I * (-self.Kd*omegaM - self.Kp*aErr)
        return self.Torque