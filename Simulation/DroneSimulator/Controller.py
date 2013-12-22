#from qmath.qmathcore import quaternion
from mathclasses import Quaternion, Vector

class PSquare(object):
    '''
    classdocs
    '''
    
    QRef = Quaternion
    PQ = None
    POmega = None
    I = Vector
    Torque = Vector


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
    
    def setPs(self, q, omega):
        self.PQ = q
        self.POmega = omega
        
    def setQRef(self, q):
        self.QRef = q
    
    def computePP(self, qM, omegaM):
        print "QRef"
        print self.QRef
        print "qM"
        print qM
        
        qErr = self.QRef.conj() * qM
        print "qErr"
        print qErr
        if qErr[0]<0:
            axisErr = -Vector(qErr)
        else:
            axisErr = Vector(qErr)
        
        print "Err"
        print axisErr
        self.Torque = -axisErr*self.PQ - omegaM*self.POmega
        #self.Torque[0] = -axisErr[0]*self.PQ - omegaM[0]*self.POmega
        #self.Torque[1] = -axisErr[1]*self.PQ - omegaM[1]*self.POmega
        #self.Torque[2] = -axisErr[2]*self.PQ - omegaM[2]*self.POmega
        
        return self.Torque