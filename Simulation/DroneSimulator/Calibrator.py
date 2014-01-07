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
    
    def newPoint(self, p, omega, alpha):
        self.fP.append(p)
        self.Iomega.append(omega)
        self.Ialpha.append(alpha)

    def calibrateI(self):
        if len(self.fP)<2:
            return False
        
        #This works if the time difference between both measurement is big enough and time step small enough and avoid angles closse to k*pi
        I1 = (self.Ialpha[1][0]-self.Ialpha[0][0])/(self.Iomega[1][1]*self.Iomega[1][2]-self.Iomega[0][1]*self.Iomega[0][2])
        I2 = (self.Ialpha[1][1]-self.Ialpha[0][1])/(self.Iomega[1][0]*self.Iomega[1][2]-self.Iomega[0][0]*self.Iomega[0][2])
        I3 = (self.Ialpha[1][2]-self.Ialpha[0][2])/(self.Iomega[1][0]*self.Iomega[1][1]-self.Iomega[0][0]*self.Iomega[0][1])
        #print "Estimated I1 " + str(I1)
        #print "Estimated I2 " + str(I2)
        #print "Estimated I3 " + str(I3)
        if self.fI.mag() == 0:
            self.fI = Vector([I1,I2,I3])
        else:
            self.fI = self.fI+Vector([I1,I2,I3])
        self.number += 1
    
    def calibrateR(self,motor,K):
        I = self.fI/float(self.number)
        
        #This works if I is well known and simulation step is small enough so that the difference between oldomega and omega is small
        Rx = (self.Ialpha[0][0] - self.Iomega[0][1]*self.Iomega[0][2]*I[0])/pow(self.fP[0],2)
        Ry = (self.Ialpha[0][1] + self.Iomega[0][0]*self.Iomega[0][2]*I[1])/pow(self.fP[0],2)
        Rz = (self.Ialpha[0][2] - self.Iomega[0][0]*self.Iomega[0][1]*I[2])/pow(self.fP[0],2)
        #print "Estimated Rx " + str(Rx)
        #print "Estimated Ry " + str(Ry)
        #print "Estimated Ry " + str(Rz)
        
        self.R[motor] = [Rx,Ry,Rz,K]
        
        return True
    
    def getAveragedI(self):
        return self.fI/float(self.number)
    
    def getIAxis(self):
        Ix = 0.5
        Iy = Ix*(1-self.fI[0]*self.fI[1])/(self.fI[1]+1)
        Iz = Ix*(self.fI[0]*self.fI[1]*self.fI[2] + self.fI[1]+1-self.fI[2])/(self.fI[1]+1)
        
        return Vector([Ix,Iy,Iz])
    
    def clearPoints(self):
        self.Ialpha = []
        self.Iomega = []
        self.fP = []
    
