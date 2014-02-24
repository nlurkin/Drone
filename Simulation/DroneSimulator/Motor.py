from ParamsClass import Params
from numpy.ma.core import sqrt

class Motor(object):
    '''
    classdocs
    '''
    
    #Calcules
    Tau_z = None #scalaire
    B = None #scalaire
    Tau_D = None #scalaire
    Power = None #scalaire
    Thrust = None #scalaire
    K = None #scalaire

    #Physical constants to set    
    Rho = None #air density #scalaire
    K_v = None #fem-omega #scalaire
    K_t = None #torque-current #scalaire
    K_tau = None #torque-thrust #scalaire
    I_M = None #Motor Intertial moments #scalar
    A_swept = None #Area swept out by rotor #scalaire
    A_xsec = None #propeller xsec #scalaire
    Radius = None #propeller radius #scalaire
    C_D = None #dimensionless constant #scalaire
    
    #Parameters
    Rotation = None #(+1 ou -1) (clockwise or anti-clockwise)  #scalaire
    
    #mesure par le moteur 
    Omega = None #motor angular velocity #scalaire
    Alpha = None #motor angular acceleration #scalaire
    
    MaxOmega = Params.MaxOmega
    
    def __init__(self):
        '''
        Constructor
        '''
    
    def setConstants(self, Rho, K_v, K_t, K_tau, I_M, A_swept, A_xsec, Radius, C_D):
        self.Rho = Rho
        self.K_v = K_v
        self.K_t = K_t
        self.K_tau = K_tau
        self.I_M = I_M
        self.A_swept = A_swept
        self.A_xsec = A_xsec
        self.Radius = Radius
        self.C_D = C_D
        self.Omega = 0
        self.Alpha = 0
        #k
        self.computeK()
        self.computeB()
    
    def setParams(self, Rotation):
        self.Rotation = Rotation
        
    def setMeasure(self, power, dt):
        if power<0:
            self.Power = 0
        else:
            self.Power = sqrt(power)
        #Omega & Alpha
        self.computeOmega(dt)
        #thrust
        self.computeThrust()
        #tau_d
        self.computeTauD()
        #tau_z
        self.computeTauZ()
    
    def computeK(self):
        self.K = pow(self.K_v*self.K_tau*sqrt(2*self.Rho*self.A_swept)/self.K_t, 2)

    def computeB(self):
        self.B = (self.Radius*self.Rho*self.C_D*self.A_xsec*pow(self.Radius, 2)/2.)
        print "B " + str(self.B)
        
    def computeOmega(self, dt):
        newOmega = self.MaxOmega * self.Power/100.
        if self.Power<0:
            newOmega = 0
        #else:
            #newOmega = sqrt(self.Power)
        if newOmega>self.MaxOmega:
            newOmega = self.MaxOmega
        
        self.Alpha = (newOmega - self.Omega)/dt
        self.Omega = newOmega
        #print "omega " + str(self.Omega)
    
    def computeThrust(self):
        self.Thrust = self.K*pow(self.Omega, 2)
        
    def computeTauD(self):
        self.Tau_D = self.B*pow(self.Omega, 2)

    def computeTauZ(self):
        #Mettre le -1 en global ou pas?
        self.Tau_z = self.Rotation*self.Tau_D + self.I_M*self.Alpha     
