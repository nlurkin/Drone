from mathclasses import Matrix, Vector


class Calibrator:
    '''
    classdocs
    '''
    fI = Matrix()
    R = [Vector(),Vector(),Vector(),Vector()]

    fP = [[]]*4
    fOmega = [[]]*4
    fAlpha = [[]]*4
    
    Iomega = Vector()
    Ialpha = Vector()

    def __init__(self):
        '''
        Constructor
        '''
    def newOmega(self,motor, omega):
        self.Iomega.append(omega)
    
    def newPoint(self, motor, p, omega, alpha):
        self.fP[motor].append(p)
        self.fOmega[motor].append(omega)
        self.fAlpha[motor].append(alpha)
        
        return self.calibrateIndividual(motor)
    
    def estimateI(self,motor,alpha,omega):
        #This works if the time difference between both measurement is big enough and time step small enough
        print "Ialpha " + str(self.Ialpha)
        print "alpha " + str(alpha)
        print "IOmega " + str(self.Iomega)
        print "omega " + str(omega)
        
        I = (alpha[0]-self.Ialpha[0])/(omega[1]*omega[2]-self.Iomega[1]*self.Iomega[2])
        print "estimated I " + str(I)
        self.Ialpha = alpha
        self.Iomega = omega
        
    def calibrate(self):
        return True

    def calibrateIndividual(self, motor):
        if len(self.fP[motor])<2:
            return False
        
        fracsq = pow(self.fP[motor][0]/self.fP[motor][1],2)
        omegaprod1 = self.fOmega[motor][0][1]*self.fOmega[motor][0][2]
        omegaprod2 = self.fOmega[motor][1][1]*self.fOmega[motor][1][2]
        I = (self.fAlpha[motor][0][0]*fracsq-self.fAlpha[motor][1][0])/(omegaprod2-omegaprod1*fracsq)
        
        
        print "Estimated I " + str(I)
        I = 0.887005649718
        #This works if I is well known and simulation step is small enough so that the difference between oldomega and omega is small
        Rx = (self.fAlpha[motor][0][0] + omegaprod1*I)/pow(self.fP[motor][0],2)
        print "Estimated Rx " + str(Rx)
        return True
        
    def clearPoints(self):
        self.fP = [[]]*4
        self.fOmega = [[]]*4
        self.fAlpha = [[]]*4
    
