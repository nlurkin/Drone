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
        
        print omega[1]*omega[2]/(p*p)
        
        return self.calibrateIndividual(motor)
    
    def estimateI(self,motor,alpha,omega):
        print "Ialpha " + str(self.Ialpha)
        print "IOmega " + str(self.Iomega)
        I = (self.Ialpha[0]-alpha[0])/(omega[1]*omega[2]-self.Iomega[1]*self.Iomega[2])
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
        #utiiliser le ancien omega si necessaire
        Rx = (self.fAlpha[motor][0][0] + omegaprod1*I)/pow(self.fP[motor][0],2)
        print "Estimated Rx " + str(Rx)
        return
        a = Matrix([12,12])
        b = Vector([0]*12)
        x = Vector([0]*12)
    
        for i in range(0,4):
            for j in range(0,3):
                b[j+3*i] = 0
                x[j+3*i] = 0
            
            a[0+3*i][0] = self.fAlpha[motor][i][0]
            a[0+3*i][1] = self.fAlpha[motor][i][1]
            a[0+3*i][2] = self.fAlpha[motor][i][2]
            a[0+3*i][3] = -self.fOmega[motor][i][0]*self.fOmega[motor][i][2]
            a[0+3*i][4] = -self.fOmega[motor][i][1]*self.fOmega[motor][i][2]
            a[0+3*i][5] = -self.fOmega[motor][i][2]*self.fOmega[motor][i][2]
            a[0+3*i][6] = self.fOmega[motor][i][0]*self.fOmega[motor][i][1]
            a[0+3*i][7] = self.fOmega[motor][i][1]*self.fOmega[motor][i][1]
            a[0+3*i][8] = self.fOmega[motor][i][2]*self.fOmega[motor][i][1]
            a[0+3*i][9] = self.fP[motor][i]
            a[0+3*i][10] = 0
            a[0+3*i][11] = 0
            a[1+3*i][0] = self.fOmega[motor][i][0]*self.fOmega[motor][i][2]
            a[1+3*i][1] = self.fOmega[motor][i][1]*self.fOmega[motor][i][2]
            a[1+3*i][2] = self.fOmega[motor][i][2]*self.fOmega[motor][i][2]
            a[1+3*i][3] = self.fAlpha[motor][i][0]
            a[1+3*i][4] = self.fAlpha[motor][i][1]
            a[1+3*i][5] = self.fAlpha[motor][i][2]
            a[1+3*i][6] = -self.fOmega[motor][i][0]*self.fOmega[motor][i][0]
            a[1+3*i][7] = -self.fOmega[motor][i][1]*self.fOmega[motor][i][0]
            a[1+3*i][8] = -self.fOmega[motor][i][2]*self.fOmega[motor][i][0]
            a[1+3*i][9] = 0
            a[1+3*i][10] = self.fP[motor][i]
            a[1+3*i][11] = 0
            a[2+3*i][0] = -self.fOmega[motor][i][0]*self.fOmega[motor][i][1]
            a[2+3*i][1] = -self.fOmega[motor][i][1]*self.fOmega[motor][i][1]
            a[2+3*i][2] = -self.fOmega[motor][i][2]*self.fOmega[motor][i][1]
            a[2+3*i][3] = self.fOmega[motor][i][0]*self.fOmega[motor][i][0]
            a[2+3*i][4] = self.fOmega[motor][i][1]*self.fOmega[motor][i][0]
            a[2+3*i][5] = self.fOmega[motor][i][2]*self.fOmega[motor][i][0]
            a[2+3*i][6] = self.fAlpha[motor][i][0]
            a[2+3*i][7] = self.fAlpha[motor][i][1]
            a[2+3*i][8] = self.fAlpha[motor][i][2]
            a[2+3*i][9] = 0
            a[2+3*i][10] = 0
            a[2+3*i][11] = self.fP[motor][i]
    
        if a.invert()!=False:
            x = a*b
            print x
            self.fI.setitem(0,0,x[0])
            self.fI.setitem(0,1,x[1])
            self.fI.setitem(0,2,x[2])
            self.fI.setitem(1,0,x[3])
            self.fI.setitem(1,1,x[4])
            self.fI.setitem(1,2,x[5])
            self.fI.setitem(2,0,x[6])
            self.fI.setitem(2,1,x[7])
            self.fI.setitem(2,2,x[8])
            self.R[motor][0] = x[9]
            self.R[motor][1] = x[10]
            self.R[motor][2] = x[11]
    
            return True
        
        return False

    def clearPoints(self):
        self.fP = [[]]*4
        self.fOmega = [[]]*4
        self.fAlpha = [[]]*4
    