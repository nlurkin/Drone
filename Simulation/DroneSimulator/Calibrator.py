from mathclasses import Matrix, Vector, Quaternion


class Calibrator:
    '''
    classdocs
    '''
    fI = Vector()
    R = [None,None,None,None]
    
    fP = []
    Iomega = []
    Ialpha = []
    Ia = []
    Iq = []
    number = 0

    def __init__(self):
        '''
        Constructor
        '''
    
    def newPoint(self, p, omega, alpha, a, q):
        self.fP.append(p)
        self.Iomega.append(omega)
        self.Ialpha.append(alpha)
        self.Ia.append(a)
        self.Iq.append(q)

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
        #OI = [0.177, 0.177, 0.334]
        #I = [(OI[1]-OI[2])/OI[0], -(OI[0]-OI[2])/OI[1], (OI[0]-OI[1])/OI[2]]
        
        #This works if I is well known and simulation step is small enough so that the difference between oldomega and omega is small
        
        Rx = (self.Ialpha[0][0] - self.Iomega[0][1]*self.Iomega[0][2]*I[0])/self.fP[0]#pow(self.fP[0],2)
        #Rx = ((self.Ialpha[1][0]-self.Ialpha[0][0]) - I[0]*(self.Iomega[1][1]*self.Iomega[1][2] - self.Iomega[0][1]*self.Iomega[0][2]))/(pow(self.fP[1],2)-pow(self.fP[0], 2))
        Ry = (self.Ialpha[0][1] - self.Iomega[0][0]*self.Iomega[0][2]*I[1])/self.fP[0]#pow(self.fP[0],2)
        #Ry = ((self.Ialpha[1][1]-self.Ialpha[0][1]) - I[1]*(self.Iomega[1][0]*self.Iomega[1][2] - self.Iomega[0][0]*self.Iomega[0][2]))/(pow(self.fP[1],2)-pow(self.fP[0], 2))
        Rz = (self.Ialpha[0][2] - self.Iomega[0][0]*self.Iomega[0][1]*I[2])/self.fP[0]#pow(self.fP[0],2)
        #Rz = ((self.Ialpha[1][2]-self.Ialpha[0][2]) - I[2]*(self.Iomega[1][0]*self.Iomega[1][1] - self.Iomega[0][0]*self.Iomega[0][1]))/(pow(self.fP[1],2)-pow(self.fP[0], 2))
        #print "Estimated Rx " + str(Rx)
        #print "Estimated Ry " + str(Ry)
        #print "Estimated Ry " + str(Rz)

        
        q1 = self.Iq[0]
        a1 = self.Ia[0]
        a1 = a1.rotate(q1.conj())
        q2 = self.Iq[1]
        a2 = self.Ia[1]
        a2 = a2.rotate(q2.conj())
        
        g = Vector([0,0, 9.81])
        g1 = g.rotate(q1.conj())
        g2 = g.rotate(q2.conj())
        
        #Ok but needs to be with very small velocity to neglect friction
        Rt = (K*(a1- a2) + K*(g1-g2))/(self.fP[0]-self.fP[1])
        
        self.R[motor] = [Rx,Ry,Rz,Rt[2]]
        
        return True
    
    def getAveragedI(self):
        return self.fI/float(self.number)
    
    def getIAxis(self):
        
        I = self.getAveragedI()
        Ix = 0.177
        Iy = Ix*(I[0]*I[1]*I[2] + I[1]+1-I[2])/(I[1]+1)
        Iz = Ix*(1+I[0]*I[1])/(I[1]+1)
        
        return Vector([Ix,Iy,Iz])
        #return [0.177, 0.177, 0.334]
    
    def getR(self,i):
        R = self.R[i][:]
        #print R
        R[0] = abs(R[0])*self.getIAxis()[0]
        R[1] = abs(R[1])*self.getIAxis()[1]
        R[2] = abs(R[2])*self.getIAxis()[2]
        
        return R
        
    def clearPoints(self):
        self.Ialpha = []
        self.Iomega = []
        self.fP = []
        self.Ia = []
        self.Iq = []
    
