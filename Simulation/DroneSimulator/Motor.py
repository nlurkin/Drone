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
    
    def setParams(self, Rotation):
        self.Rotation = Rotation
        
    def setMeasure(self, Omega, Alpha):
        self.Omega = Omega
        self.Alpha = Alpha

    
    def tau_z(self):
        #Mettre le -1 en global ou pas?
        self.Tau_z = self.Rotation*self.tau_d() + self.I_M*self.Alpha
        return self.Tau_z 
    
    def tau_d(self):
        self.Tau_D = self.b()*pow(self.Omega, 2)
        return self.Tau_D
        
    def b(self):
        self.B = (self.Radius*self.Rho*self.C_D*self.A_xsec*pow(self.Radius, 2)/2.)
        return self.B
    
    def power(self):
        self.Power = (self.K_v/self.K_t)*self.Tau*self.Omega
        return self.Power 
    
    def thrust(self):
        self.Thrust = self.k()*pow(self.Omega, 2)
        return self.Thrust
        
    def k(self):
        self.K = pow(self.K_v*self.K_tau*sqrt(2*self.Rho*self.A_swept)/self.K_t, 2)
        return self.K 
    