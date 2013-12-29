#from qmath.qmathcore import quaternion
from ParamsClass import Params
from mathclasses import Matrix, Quaternion, Vector

class PSquare:
    '''
    classdocs
    '''
    
    QRef = Quaternion()
    ARef = Vector()
    PQ = None
    POmega = None
    I = Matrix()
    Torque = Vector()
    
    Km1 = None
    Km2 = None
    Km3 = None
    Km4 = None
    
    Output = None


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
        self.Output = Vector([0,0,0,0])
        self.ARef = Vector([0,0,0])
    
    def setI(self, I):
        self.I = Matrix([[I[0],0,0],[0,I[1],0],[0,0,I[2]]])
    
    def setPs(self, v):
        self.PQ = v[0]
        self.POmega = v[1]
        
    def setQRef(self, q, a):
        self.QRef = q
        self.ARef = a
    
    def setMotorCoefficient(self, m1, m2, m3, m4):
        self.Km1 = m1
        self.Km2 = m2
        self.Km3 = m3
        self.Km4 = m4
        
    def computePP(self, qM, omegaM, aM):
        qErr = self.QRef * qM.conj()
        if qErr[0]<0:
            axisErr = -Vector(qErr)
        else:
            axisErr = Vector(qErr)
        
        self.Torque = self.I*(axisErr*self.PQ - omegaM*self.POmega)
        
        if Params.TorqueIsSet:
            return self.Torque
        
        T = self.ARef.rotate(qM)[2]
        '''p1 = (self.Km3[0]*(self.Km2[1]*(self.Km4[4]*self.Torque[2]-self.Km4[3]*T)+self.Km4[1]*(self.Km2[2]*T-self.Km2[4]*self.Torque[2])+self.Torque[1]*(self.Km2[4]*self.Km4[3]-self.Km4[4]*self.Km2[2]))+self.Torque[0]*(self.Km2[1]*(self.Km3[4]*self.Km4[3]-self.Km4[4]*self.Km3[2])+self.Km4[1]*(self.Km2[4]*self.Km3[2]-self.Km3[4]*self.Km2[2])))/(self.Km1[0]*(self.Km2[1]*(self.Km3[4]*self.Km4[3]-self.Km4[4]*self.Km3[2])+self.Km4[1]*(self.Km2[4]*self.Km3[2]-self.Km3[4]*self.Km2[2]))+self.Km3[0]*(self.Km2[1]*(self.Km4[4]*self.Km1[2]-self.Km1[4]*self.Km4[3])+self.Km4[1]*(self.Km1[4]*self.Km2[2]-self.Km2[4]*self.Km1[2])))
        p2=-(self.Km1[0]*(self.Km4[1]*(self.Km3[4]*self.Torque[2]-self.Km3[2]*T)+self.Torque[1]*(self.Km4[4]*self.Km3[2]-self.Km3[4]*self.Km4[2]))+self.Km3[0]*(self.Km4[1]*(self.Km1[2]*T-self.Km1[4]*self.Torque[2])+self.Torque[1]*(self.Km1[4]*self.Km4[2]-self.Km4[4]*self.Km1[2]))+self.Torque[0]*self.Km4[1]*(self.Km1[4]*self.Km3[2]-self.Km3[4]*self.Km1[2]))/(self.Km1[0]*(self.Km2[1]*(self.Km3[4]*self.Km4[2]-self.Km4[4]*self.Km3[2])+self.Km4[1]*(self.Km2[4]*self.Km3[2]-self.Km3[4]*self.Km2[2]))+self.Km3[0]*(self.Km2[1]*(self.Km4[4]*self.Km1[2]-self.Km1[4]*self.Km4[2])+self.Km4[1]*(self.Km1[4]*self.Km2[2]-self.Km2[4]*self.Km1[2])))
        p3=-(self.Km1[0]*(self.Km2[1]*(self.Km4[4]*self.Torque[2]-self.Km4[2]*T)+self.Km4[1]*(self.Km2[2]*T-self.Km2[4]*self.Torque[2])+self.Torque[1]*(self.Km2[4]*self.Km4[2]-self.Km4[4]*self.Km2[2]))+self.Torque[0]*(self.Km2[1]*(self.Km1[4]*self.Km4[2]-self.Km4[4]*self.Km1[2])+self.Km4[1]*(self.Km2[4]*self.Km1[2]-self.Km1[4]*self.Km2[2])))/(self.Km1[0]*(self.Km2[1]*(self.Km3[4]*self.Km4[2]-self.Km4[4]*self.Km3[2])+self.Km4[1]*(self.Km2[4]*self.Km3[2]-self.Km3[4]*self.Km2[2]))+self.Km3[0]*(self.Km2[1]*(self.Km4[4]*self.Km1[2]-self.Km1[4]*self.Km4[2])+self.Km4[1]*(self.Km1[4]*self.Km2[2]-self.Km2[4]*self.Km1[2])))
        p4=(self.Km1[0]*(self.Km2[1]*(self.Km3[4]*self.Torque[2]-self.Km3[2]*T)+self.Torque[1]*(self.Km2[4]*self.Km3[2]-self.Km3[4]*self.Km2[2]))+self.Km3[0]*(self.Km2[1]*(self.Km1[2]*T-self.Km1[4]*self.Torque[2])+self.Torque[1]*(self.Km1[4]*self.Km2[2]-self.Km2[4]*self.Km1[2]))+self.Torque[0]*self.Km2[1]*(self.Km1[4]*self.Km3[2]-self.Km3[4]*self.Km1[2]))/(self.Km1[0]*(self.Km2[1]*(self.Km3[4]*self.Km4[2]-self.Km4[4]*self.Km3[2])+self.Km4[1]*(self.Km2[4]*self.Km3[2]-self.Km3[4]*self.Km2[2]))+self.Km3[0]*(self.Km2[1]*(self.Km4[4]*self.Km1[2]-self.Km1[4]*self.Km4[2])+self.Km4[1]*(self.Km1[4]*self.Km2[2]-self.Km2[4]*self.Km1[2])))'''
        #(L*(b*T+k*self.Torque[2])+2*b*self.Torque[0])/(4*b*k*L),p2=-(L*(k*self.Torque[2]-b*T)-2*b*self.Torque[1])/(4*b*k*L),p3=(L*(b*T+k*self.Torque[2])-2*b*self.Torque[0])/(4*b*k*L),p4=-(L*(k*self.Torque[2]-b*T)+2*b*self.Torque[1])/(4*b*k*L)
        '''p1=((self.Torque[1]*(self.Km4[2]*self.Km3[3]-self.Km3[2]*self.Km4[3])+self.Km3[1]*(self.Torque[2]*self.Km4[3]-self.Km4[2]*T)+self.Km4[1]*(self.Km3[2]*T-self.Torque[2]*self.Km3[3]))*self.Km2[0]+self.Torque[0]*(self.Km2[1]*(self.Km3[2]*self.Km4[3]-self.Km4[2]*self.Km3[3])+self.Km3[1]*(self.Km4[2]*self.Km2[3]-self.Km2[2]*self.Km4[3])+self.Km4[1]*(self.Km2[2]*self.Km3[3]-self.Km3[2]*self.Km2[3]))+self.Km3[0]*(self.Torque[1]*(self.Km2[2]*self.Km4[3]-self.Km4[2]*self.Km2[3])+self.Km2[1]*(self.Km4[2]*T-self.Torque[2]*self.Km4[3])+self.Km4[1]*(self.Torque[2]*self.Km2[3]-self.Km2[2]*T))+self.Km4[0]*(self.Torque[1]*(self.Km3[2]*self.Km2[3]-self.Km2[2]*self.Km3[3])+self.Km2[1]*(self.Torque[2]*self.Km3[3]-self.Km3[2]*T)+self.Km3[1]*(self.Km2[2]*T-self.Torque[2]*self.Km2[3])))/((self.Km1[1]*(self.Km4[2]*self.Km3[3]-self.Km3[2]*self.Km4[3])+self.Km3[1]*(self.Km1[2]*self.Km4[3]-self.Km4[2]*self.Km1[3])+self.Km4[1]*(self.Km3[2]*self.Km1[3]-self.Km1[2]*self.Km3[3]))*self.Km2[0]+self.Km1[0]*(self.Km2[1]*(self.Km3[2]*self.Km4[3]-self.Km4[2]*self.Km3[3])+self.Km3[1]*(self.Km4[2]*self.Km2[3]-self.Km2[2]*self.Km4[3])+self.Km4[1]*(self.Km2[2]*self.Km3[3]-self.Km3[2]*self.Km2[3]))+self.Km3[0]*(self.Km1[1]*(self.Km2[2]*self.Km4[3]-self.Km4[2]*self.Km2[3])+self.Km2[1]*(self.Km4[2]*self.Km1[3]-self.Km1[2]*self.Km4[3])+self.Km4[1]*(self.Km1[2]*self.Km2[3]-self.Km2[2]*self.Km1[3]))+self.Km4[0]*(self.Km1[1]*(self.Km3[2]*self.Km2[3]-self.Km2[2]*self.Km3[3])+self.Km2[1]*(self.Km1[2]*self.Km3[3]-self.Km3[2]*self.Km1[3])+self.Km3[1]*(self.Km2[2]*self.Km1[3]-self.Km1[2]*self.Km2[3])))
        p2=-(self.Torque[0]*(self.Km1[1]*(self.Km3[2]*self.Km4[3]-self.Km4[2]*self.Km3[3])+self.Km3[1]*(self.Km4[2]*self.Km1[3]-self.Km1[2]*self.Km4[3])+self.Km4[1]*(self.Km1[2]*self.Km3[3]-self.Km3[2]*self.Km1[3]))+self.Km1[0]*(self.Torque[1]*(self.Km4[2]*self.Km3[3]-self.Km3[2]*self.Km4[3])+self.Km3[1]*(self.Torque[2]*self.Km4[3]-self.Km4[2]*T)+self.Km4[1]*(self.Km3[2]*T-self.Torque[2]*self.Km3[3]))+self.Km3[0]*(self.Torque[1]*(self.Km1[2]*self.Km4[3]-self.Km4[2]*self.Km1[3])+self.Km1[1]*(self.Km4[2]*T-self.Torque[2]*self.Km4[3])+self.Km4[1]*(self.Torque[2]*self.Km1[3]-self.Km1[2]*T))+self.Km4[0]*(self.Torque[1]*(self.Km3[2]*self.Km1[3]-self.Km1[2]*self.Km3[3])+self.Km1[1]*(self.Torque[2]*self.Km3[3]-self.Km3[2]*T)+self.Km3[1]*(self.Km1[2]*T-self.Torque[2]*self.Km1[3])))/((self.Km1[1]*(self.Km4[2]*self.Km3[3]-self.Km3[2]*self.Km4[3])+self.Km3[1]*(self.Km1[2]*self.Km4[3]-self.Km4[2]*self.Km1[3])+self.Km4[1]*(self.Km3[2]*self.Km1[3]-self.Km1[2]*self.Km3[3]))*self.Km2[0]+self.Km1[0]*(self.Km2[1]*(self.Km3[2]*self.Km4[3]-self.Km4[2]*self.Km3[3])+self.Km3[1]*(self.Km4[2]*self.Km2[3]-self.Km2[2]*self.Km4[3])+self.Km4[1]*(self.Km2[2]*self.Km3[3]-self.Km3[2]*self.Km2[3]))+self.Km3[0]*(self.Km1[1]*(self.Km2[2]*self.Km4[3]-self.Km4[2]*self.Km2[3])+self.Km2[1]*(self.Km4[2]*self.Km1[3]-self.Km1[2]*self.Km4[3])+self.Km4[1]*(self.Km1[2]*self.Km2[3]-self.Km2[2]*self.Km1[3]))+self.Km4[0]*(self.Km1[1]*(self.Km3[2]*self.Km2[3]-self.Km2[2]*self.Km3[3])+self.Km2[1]*(self.Km1[2]*self.Km3[3]-self.Km3[2]*self.Km1[3])+self.Km3[1]*(self.Km2[2]*self.Km1[3]-self.Km1[2]*self.Km2[3])))
        p3=-((self.Torque[1]*(self.Km4[2]*self.Km1[3]-self.Km1[2]*self.Km4[3])+self.Km1[1]*(self.Torque[2]*self.Km4[3]-self.Km4[2]*T)+self.Km4[1]*(self.Km1[2]*T-self.Torque[2]*self.Km1[3]))*self.Km2[0]+self.Km1[0]*(self.Torque[1]*(self.Km2[2]*self.Km4[3]-self.Km4[2]*self.Km2[3])+self.Km2[1]*(self.Km4[2]*T-self.Torque[2]*self.Km4[3])+self.Km4[1]*(self.Torque[2]*self.Km2[3]-self.Km2[2]*T))+self.Torque[0]*(self.Km1[1]*(self.Km4[2]*self.Km2[3]-self.Km2[2]*self.Km4[3])+self.Km2[1]*(self.Km1[2]*self.Km4[3]-self.Km4[2]*self.Km1[3])+self.Km4[1]*(self.Km2[2]*self.Km1[3]-self.Km1[2]*self.Km2[3]))+self.Km4[0]*(self.Torque[1]*(self.Km1[2]*self.Km2[3]-self.Km2[2]*self.Km1[3])+self.Km1[1]*(self.Km2[2]*T-self.Torque[2]*self.Km2[3])+self.Km2[1]*(self.Torque[2]*self.Km1[3]-self.Km1[2]*T)))/((self.Km1[1]*(self.Km4[2]*self.Km3[3]-self.Km3[2]*self.Km4[3])+self.Km3[1]*(self.Km1[2]*self.Km4[3]-self.Km4[2]*self.Km1[3])+self.Km4[1]*(self.Km3[2]*self.Km1[3]-self.Km1[2]*self.Km3[3]))*self.Km2[0]+self.Km1[0]*(self.Km2[1]*(self.Km3[2]*self.Km4[3]-self.Km4[2]*self.Km3[3])+self.Km3[1]*(self.Km4[2]*self.Km2[3]-self.Km2[2]*self.Km4[3])+self.Km4[1]*(self.Km2[2]*self.Km3[3]-self.Km3[2]*self.Km2[3]))+self.Km3[0]*(self.Km1[1]*(self.Km2[2]*self.Km4[3]-self.Km4[2]*self.Km2[3])+self.Km2[1]*(self.Km4[2]*self.Km1[3]-self.Km1[2]*self.Km4[3])+self.Km4[1]*(self.Km1[2]*self.Km2[3]-self.Km2[2]*self.Km1[3]))+self.Km4[0]*(self.Km1[1]*(self.Km3[2]*self.Km2[3]-self.Km2[2]*self.Km3[3])+self.Km2[1]*(self.Km1[2]*self.Km3[3]-self.Km3[2]*self.Km1[3])+self.Km3[1]*(self.Km2[2]*self.Km1[3]-self.Km1[2]*self.Km2[3])))
        p4=((self.Torque[1]*(self.Km3[2]*self.Km1[3]-self.Km1[2]*self.Km3[3])+self.Km1[1]*(self.Torque[2]*self.Km3[3]-self.Km3[2]*T)+self.Km3[1]*(self.Km1[2]*T-self.Torque[2]*self.Km1[3]))*self.Km2[0]+self.Km1[0]*(self.Torque[1]*(self.Km2[2]*self.Km3[3]-self.Km3[2]*self.Km2[3])+self.Km2[1]*(self.Km3[2]*T-self.Torque[2]*self.Km3[3])+self.Km3[1]*(self.Torque[2]*self.Km2[3]-self.Km2[2]*T))+self.Torque[0]*(self.Km1[1]*(self.Km3[2]*self.Km2[3]-self.Km2[2]*self.Km3[3])+self.Km2[1]*(self.Km1[2]*self.Km3[3]-self.Km3[2]*self.Km1[3])+self.Km3[1]*(self.Km2[2]*self.Km1[3]-self.Km1[2]*self.Km2[3]))+self.Km3[0]*(self.Torque[1]*(self.Km1[2]*self.Km2[3]-self.Km2[2]*self.Km1[3])+self.Km1[1]*(self.Km2[2]*T-self.Torque[2]*self.Km2[3])+self.Km2[1]*(self.Torque[2]*self.Km1[3]-self.Km1[2]*T)))/((self.Km1[1]*(self.Km4[2]*self.Km3[3]-self.Km3[2]*self.Km4[3])+self.Km3[1]*(self.Km1[2]*self.Km4[3]-self.Km4[2]*self.Km1[3])+self.Km4[1]*(self.Km3[2]*self.Km1[3]-self.Km1[2]*self.Km3[3]))*self.Km2[0]+self.Km1[0]*(self.Km2[1]*(self.Km3[2]*self.Km4[3]-self.Km4[2]*self.Km3[3])+self.Km3[1]*(self.Km4[2]*self.Km2[3]-self.Km2[2]*self.Km4[3])+self.Km4[1]*(self.Km2[2]*self.Km3[3]-self.Km3[2]*self.Km2[3]))+self.Km3[0]*(self.Km1[1]*(self.Km2[2]*self.Km4[3]-self.Km4[2]*self.Km2[3])+self.Km2[1]*(self.Km4[2]*self.Km1[3]-self.Km1[2]*self.Km4[3])+self.Km4[1]*(self.Km1[2]*self.Km2[3]-self.Km2[2]*self.Km1[3]))+self.Km4[0]*(self.Km1[1]*(self.Km3[2]*self.Km2[3]-self.Km2[2]*self.Km3[3])+self.Km2[1]*(self.Km1[2]*self.Km3[3]-self.Km3[2]*self.Km1[3])+self.Km3[1]*(self.Km2[2]*self.Km1[3]-self.Km1[2]*self.Km2[3])))
        self.Output = Vector([p1,p2,p3,p4])'''
        self.Output = Vector([0,0,0,0])
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