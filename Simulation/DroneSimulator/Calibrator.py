from mathclasses import Matrix, Vector


class Calibrator:
    '''
    classdocs
    '''
    fI = Matrix()
    R = [0,0,0,0]

    fP = [[]]*4
    fOmega = [[]]*4
    fAlpha = [[]]*4

    def __init__(self, params):
        '''
        Constructor
        '''
    
    def newPoint(self, motor, p, omega, alpha):
        self.fP[motor].append(p)
        self.fOmega[motor].append(omega)
        self.fAlpha[motor].append(alpha)

        return self.calibrateIndividual(motor)

    def calibrate(self):
        return True

    def calibrateIndividual(self, motor):
        if self.fP[motor].size()<12:
            return False
    
        a = Matrix([12,12])
        b = Vector([0]*12)
        x = Vector([0]*12)
    
        for i in range(0,4):
            for j in range(0,3):
                b[j+3*i] = 0
                x[j+3*i] = 0
            
            a[0+3*i][0] = self.fAlpha[motor][i].x
            a[0+3*i][1] = self.fAlpha[motor][i].y
            a[0+3*i][2] = self.fAlpha[motor][i].z
            a[0+3*i][3] = -self.fOmega[motor][i].x*self.fOmega[motor][i].z
            a[0+3*i][4] = -self.fOmega[motor][i].y*self.fOmega[motor][i].z
            a[0+3*i][5] = -self.fOmega[motor][i].z*self.fOmega[motor][i].z
            a[0+3*i][6] = self.fOmega[motor][i].x*self.fOmega[motor][i].y
            a[0+3*i][7] = self.fOmega[motor][i].y*self.fOmega[motor][i].y
            a[0+3*i][8] = self.fOmega[motor][i].z*self.fOmega[motor][i].y
            a[0+3*i][9] = self.fP[motor][i]
            a[0+3*i][10] = 0
            a[0+3*i][11] = 0
            a[1+3*i][0] = self.fOmega[motor][i].x*self.fOmega[motor][i].z
            a[1+3*i][1] = self.fOmega[motor][i].y*self.fOmega[motor][i].z
            a[1+3*i][2] = self.fOmega[motor][i].z*self.fOmega[motor][i].z
            a[1+3*i][3] = self.fAlpha[motor][i].x
            a[1+3*i][4] = self.fAlpha[motor][i].y
            a[1+3*i][5] = self.fAlpha[motor][i].z
            a[1+3*i][6] = -self.fOmega[motor][i].x*self.fOmega[motor][i].x
            a[1+3*i][7] = -self.fOmega[motor][i].y*self.fOmega[motor][i].x
            a[1+3*i][8] = -self.fOmega[motor][i].z*self.fOmega[motor][i].x
            a[1+3*i][9] = 0
            a[1+3*i][10] = self.fP[motor][i]
            a[1+3*i][11] = 0
            a[2+3*i][0] = -self.fOmega[motor][i].x*self.fOmega[motor][i].y
            a[2+3*i][1] = -self.fOmega[motor][i].y*self.fOmega[motor][i].y
            a[2+3*i][2] = -self.fOmega[motor][i].z*self.fOmega[motor][i].y
            a[2+3*i][3] = self.fOmega[motor][i].x*self.fOmega[motor][i].x
            a[2+3*i][4] = self.fOmega[motor][i].y*self.fOmega[motor][i].x
            a[2+3*i][5] = self.fOmega[motor][i].z*self.fOmega[motor][i].x
            a[2+3*i][6] = self.fAlpha[motor][i].x
            a[2+3*i][7] = self.fAlpha[motor][i].y
            a[2+3*i][8] = self.fAlpha[motor][i].z
            a[2+3*i][9] = 0
            a[2+3*i][10] = 0
            a[2+3*i][11] = self.fP[motor][i]
    
        if(Matrix.Invert(a, 12)):
            Matrix.Multiply(a, b, 12, 12, 12, x)
            fI(0,0) = x[0]
            fI(0,1) = x[1]
            fI(0,2) = x[2]
            fI(1,0) = x[3]
            fI(1,1) = x[4]
            fI(1,2) = x[5]
            fI(2,0) = x[6]
            fI(2,1) = x[7]
            fI(2,2) = x[8]
            R[motor].x = x[9]
            R[motor].y = x[10]
            R[motor].z = x[11]
    
            return True
        
        return False

    def clearPoints(self):
        self.fP = [[]]*4
        self.fOmega = [[]]*4
        self.fAlpha = [[]]*4
    