#from qmath.qmathcore import quaternion
from ParamsClass import Params
from mathclasses import Matrix, Quaternion, Vector, createRotation

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
        self.ARef = a-Params.Gravity
    
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
        
    def computePP(self, qM, omegaM, aM):
        if self.ARef[0]!=0 or self.ARef[1]!=0:
            qRef = createRotation(self.ARef, Vector([0,0,1]))
            qRef.w = 1
            qRef.normalize()
        else:
            qRef = self.QRef

        print "Aref " + self.ARef.rotate(qM)
        print "AM " + aM
        #T = (self.ARef.rotate(qM)-aM)
        #T = (self.ARef.rotate(qM))
        T = self.ARef.rotate(qM.conj())
        print "T " + str(T)
        T = T[2]*self.Mass

        print "qM " + qM
        print "omegaM " + omegaM
        print "qRef " + qRef
        qErr = qRef * qM.conj()

        if qErr[0]<0:
            axisErr = -Vector(qErr)
        else:
            axisErr = Vector(qErr)
        
        self.Torque = self.I*(axisErr*self.PQ - omegaM*self.POmega)
        
        if Params.TorqueIsSet:
            return self.Torque
        print "T " + str(T)
        '''p1 = (self.Km3[0]*(self.Km2[1]*(self.Km4[4]*self.Torque[2]-self.Km4[3]*T)+self.Km4[1]*(self.Km2[2]*T-self.Km2[4]*self.Torque[2])+self.Torque[1]*(self.Km2[4]*self.Km4[3]-self.Km4[4]*self.Km2[2]))+self.Torque[0]*(self.Km2[1]*(self.Km3[4]*self.Km4[3]-self.Km4[4]*self.Km3[2])+self.Km4[1]*(self.Km2[4]*self.Km3[2]-self.Km3[4]*self.Km2[2])))/(self.Km1[0]*(self.Km2[1]*(self.Km3[4]*self.Km4[3]-self.Km4[4]*self.Km3[2])+self.Km4[1]*(self.Km2[4]*self.Km3[2]-self.Km3[4]*self.Km2[2]))+self.Km3[0]*(self.Km2[1]*(self.Km4[4]*self.Km1[2]-self.Km1[4]*self.Km4[3])+self.Km4[1]*(self.Km1[4]*self.Km2[2]-self.Km2[4]*self.Km1[2])))
        p2=-(self.Km1[0]*(self.Km4[1]*(self.Km3[4]*self.Torque[2]-self.Km3[2]*T)+self.Torque[1]*(self.Km4[4]*self.Km3[2]-self.Km3[4]*self.Km4[2]))+self.Km3[0]*(self.Km4[1]*(self.Km1[2]*T-self.Km1[4]*self.Torque[2])+self.Torque[1]*(self.Km1[4]*self.Km4[2]-self.Km4[4]*self.Km1[2]))+self.Torque[0]*self.Km4[1]*(self.Km1[4]*self.Km3[2]-self.Km3[4]*self.Km1[2]))/(self.Km1[0]*(self.Km2[1]*(self.Km3[4]*self.Km4[2]-self.Km4[4]*self.Km3[2])+self.Km4[1]*(self.Km2[4]*self.Km3[2]-self.Km3[4]*self.Km2[2]))+self.Km3[0]*(self.Km2[1]*(self.Km4[4]*self.Km1[2]-self.Km1[4]*self.Km4[2])+self.Km4[1]*(self.Km1[4]*self.Km2[2]-self.Km2[4]*self.Km1[2])))
        p3=-(self.Km1[0]*(self.Km2[1]*(self.Km4[4]*self.Torque[2]-self.Km4[2]*T)+self.Km4[1]*(self.Km2[2]*T-self.Km2[4]*self.Torque[2])+self.Torque[1]*(self.Km2[4]*self.Km4[2]-self.Km4[4]*self.Km2[2]))+self.Torque[0]*(self.Km2[1]*(self.Km1[4]*self.Km4[2]-self.Km4[4]*self.Km1[2])+self.Km4[1]*(self.Km2[4]*self.Km1[2]-self.Km1[4]*self.Km2[2])))/(self.Km1[0]*(self.Km2[1]*(self.Km3[4]*self.Km4[2]-self.Km4[4]*self.Km3[2])+self.Km4[1]*(self.Km2[4]*self.Km3[2]-self.Km3[4]*self.Km2[2]))+self.Km3[0]*(self.Km2[1]*(self.Km4[4]*self.Km1[2]-self.Km1[4]*self.Km4[2])+self.Km4[1]*(self.Km1[4]*self.Km2[2]-self.Km2[4]*self.Km1[2])))
        p4=(self.Km1[0]*(self.Km2[1]*(self.Km3[4]*self.Torque[2]-self.Km3[2]*T)+self.Torque[1]*(self.Km2[4]*self.Km3[2]-self.Km3[4]*self.Km2[2]))+self.Km3[0]*(self.Km2[1]*(self.Km1[2]*T-self.Km1[4]*self.Torque[2])+self.Torque[1]*(self.Km1[4]*self.Km2[2]-self.Km2[4]*self.Km1[2]))+self.Torque[0]*self.Km2[1]*(self.Km1[4]*self.Km3[2]-self.Km3[4]*self.Km1[2]))/(self.Km1[0]*(self.Km2[1]*(self.Km3[4]*self.Km4[2]-self.Km4[4]*self.Km3[2])+self.Km4[1]*(self.Km2[4]*self.Km3[2]-self.Km3[4]*self.Km2[2]))+self.Km3[0]*(self.Km2[1]*(self.Km4[4]*self.Km1[2]-self.Km1[4]*self.Km4[2])+self.Km4[1]*(self.Km1[4]*self.Km2[2]-self.Km2[4]*self.Km1[2])))'''
        #(L*(b*T+k*self.Torque[2])+2*b*self.Torque[0])/(4*b*k*L),p2=-(L*(k*self.Torque[2]-b*T)-2*b*self.Torque[1])/(4*b*k*L),p3=(L*(b*T+k*self.Torque[2])-2*b*self.Torque[0])/(4*b*k*L),p4=-(L*(k*self.Torque[2]-b*T)+2*b*self.Torque[1])/(4*b*k*L)
        print "Torque " + self.Torque
        print self.Km1
        print self.Km2
        print self.Km3
        print self.Km4
        #T = 0
        '''p1=((self.L)*((self.b)*T+(self.Torque[2])*(self.K))+2*(self.Torque[0])*(self.b))/(4*(self.b)*(self.K)*(self.L))
        p2=-((self.L)*((self.Torque[2])*(self.K)-(self.b)*T)-2*(self.Torque[1])*(self.b))/(4*(self.b)*(self.K)*(self.L))
        p3=((self.L)*((self.b)*T+(self.Torque[2])*(self.K))-2*(self.Torque[0])*(self.b))/(4*(self.b)*(self.K)*(self.L))
        p4=-((self.L)*((self.Torque[2])*(self.K)-(self.b)*T)+2*(self.Torque[1])*(self.b))/(4*(self.b)*(self.K)*(self.L))'''
        
        deno = ((self.Km1[0])*((self.Km2[1])*((self.Km3[2])*(self.Km4[3])+(self.Km4[2])*(self.Km3[3]))+(self.Km4[1])*((self.Km2[2])*(self.Km3[3])+(self.Km3[2])*(self.Km2[3])))+(self.Km3[0])*((self.Km2[1])*((self.Km1[2])*(self.Km4[3])+(self.Km4[2])*(self.Km1[3]))+(self.Km4[1])*((self.Km1[2])*(self.Km2[3])+(self.Km2[2])*(self.Km1[3]))))
        print deno
        p1=((self.Km3[0])*((self.Km2[1])*((self.Km4[2])*T+(self.Torque[2])*(self.Km4[3]))+(self.Km4[1])*((self.Km2[2])*T+(self.Torque[2])*(self.Km2[3]))+(self.Torque[1])*((self.Km2[2])*(self.Km4[3])-(self.Km4[2])*(self.Km2[3])))+(self.Torque[0])*((self.Km2[1])*((self.Km3[2])*(self.Km4[3])+(self.Km4[2])*(self.Km3[3]))+(self.Km4[1])*((self.Km2[2])*(self.Km3[3])+(self.Km3[2])*(self.Km2[3]))))/deno
        p2=-((self.Km1[0])*((self.Km4[1])*((self.Torque[2])*(self.Km3[3])-(self.Km3[2])*T)+(self.Torque[1])*(-(self.Km3[2])*(self.Km4[3])-(self.Km4[2])*(self.Km3[3])))+(self.Km3[0])*((self.Km4[1])*((self.Torque[2])*(self.Km1[3])-(self.Km1[2])*T)+(self.Torque[1])*(-(self.Km1[2])*(self.Km4[3])-(self.Km4[2])*(self.Km1[3])))+(self.Torque[0])*(self.Km4[1])*((self.Km3[2])*(self.Km1[3])-(self.Km1[2])*(self.Km3[3])))/((self.Km1[0])*((self.Km2[1])*((self.Km3[2])*(self.Km4[3])+(self.Km4[2])*(self.Km3[3]))+(self.Km4[1])*((self.Km2[2])*(self.Km3[3])+(self.Km3[2])*(self.Km2[3])))+(self.Km3[0])*((self.Km2[1])*((self.Km1[2])*(self.Km4[3])+(self.Km4[2])*(self.Km1[3]))+(self.Km4[1])*((self.Km1[2])*(self.Km2[3])+(self.Km2[2])*(self.Km1[3]))))
        p3=-((self.Km1[0])*((self.Km2[1])*(-(self.Km4[2])*T-(self.Torque[2])*(self.Km4[3]))+(self.Km4[1])*(-(self.Km2[2])*T-(self.Torque[2])*(self.Km2[3]))+(self.Torque[1])*((self.Km4[2])*(self.Km2[3])-(self.Km2[2])*(self.Km4[3])))+(self.Torque[0])*((self.Km2[1])*((self.Km1[2])*(self.Km4[3])+(self.Km4[2])*(self.Km1[3]))+(self.Km4[1])*((self.Km1[2])*(self.Km2[3])+(self.Km2[2])*(self.Km1[3]))))/((self.Km1[0])*((self.Km2[1])*((self.Km3[2])*(self.Km4[3])+(self.Km4[2])*(self.Km3[3]))+(self.Km4[1])*((self.Km2[2])*(self.Km3[3])+(self.Km3[2])*(self.Km2[3])))+(self.Km3[0])*((self.Km2[1])*((self.Km1[2])*(self.Km4[3])+(self.Km4[2])*(self.Km1[3]))+(self.Km4[1])*((self.Km1[2])*(self.Km2[3])+(self.Km2[2])*(self.Km1[3]))))
        p4=-((self.Km1[0])*((self.Km2[1])*((self.Torque[2])*(self.Km3[3])-(self.Km3[2])*T)+(self.Torque[1])*((self.Km2[2])*(self.Km3[3])+(self.Km3[2])*(self.Km2[3])))+(self.Km3[0])*((self.Km2[1])*((self.Torque[2])*(self.Km1[3])-(self.Km1[2])*T)+(self.Torque[1])*((self.Km1[2])*(self.Km2[3])+(self.Km2[2])*(self.Km1[3])))+(self.Torque[0])*(self.Km2[1])*((self.Km3[2])*(self.Km1[3])-(self.Km1[2])*(self.Km3[3])))/((self.Km1[0])*((self.Km2[1])*((self.Km3[2])*(self.Km4[3])+(self.Km4[2])*(self.Km3[3]))+(self.Km4[1])*((self.Km2[2])*(self.Km3[3])+(self.Km3[2])*(self.Km2[3])))+(self.Km3[0])*((self.Km2[1])*((self.Km1[2])*(self.Km4[3])+(self.Km4[2])*(self.Km1[3]))+(self.Km4[1])*((self.Km1[2])*(self.Km2[3])+(self.Km2[2])*(self.Km1[3]))))
        self.Output = Vector([p1,p2,p3,p4])
        print "Output " + self.Output
        print "Tau_x " + str(self.L*self.K*(p1-p3))
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