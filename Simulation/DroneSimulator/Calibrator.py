from mathclasses import Matrix, Vector


class Calibrator:
    '''
    classdocs
    '''
    fI = Vector()
    R = [None,None,None,None]
    
    fP = []
    Iomega = []
    Ialpha = []
    number = 0

    def __init__(self):
        '''
        Constructor
        '''
    
    def newPoint(self, p, omega, alpha,motor):
        self.fP.append(p)
        self.Iomega.append(omega)
        self.Ialpha.append(alpha)
        
        return self.calibrateIndividual(motor)

    def calibrateIndividual(self, motor):
        if len(self.fP)<2:
            return False
        
        #This works if the time difference between both measurement is big enough and time step small enough
        I1 = (self.Ialpha[1][0]-self.Ialpha[0][0])/(self.Iomega[1][1]*self.Iomega[1][2]-self.Iomega[0][1]*self.Iomega[0][2])
        I2 = (self.Ialpha[1][1]-self.Ialpha[0][1])/(self.Iomega[1][0]*self.Iomega[1][2]-self.Iomega[0][0]*self.Iomega[0][2])
        I3 = (self.Ialpha[1][2]-self.Ialpha[0][2])/(self.Iomega[1][0]*self.Iomega[1][1]-self.Iomega[0][0]*self.Iomega[0][1])
        print "Estimated I1 " + str(I1)
        #print "Estimated I2 " + str(I2)
        #print "Estimated I3 " + str(I3)
        if self.fI[0] == 0:
            self.fI[0] = I1
        else:
            self.fI[0] = self.fI[0]+I1
        self.number += 1
        
        print "I moyenne " + str(self.fI[0]/self.number)
        #I1 = 0.887005649718
        #This works if I is well known and simulation step is small enough so that the difference between oldomega and omega is small
        Rx = (self.Ialpha[0][0] - self.Iomega[0][1]*self.Iomega[0][2]*I1)/pow(self.fP[0],2)
        Ry = (self.Ialpha[0][1] + self.Iomega[0][0]*self.Iomega[0][2]*I2)/pow(self.fP[0],2)
        Rz = (self.Ialpha[0][2] - self.Iomega[0][0]*self.Iomega[0][1]*I3)/pow(self.fP[0],2)
        #print "Estimated Rx " + str(Rx)
        #print "Estimated Ry " + str(Ry)
        #print "Estimated Ry " + str(Rz)
        
        self.R[motor] = Vector([Rx,Ry,Rz])
        
        return True
        
    def clearPoints(self):
        self.Ialpha = []
        self.Iomega = []
        self.fP = []
    
