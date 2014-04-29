'''
Created on 25 Apr 2014

@author: Nicolas
'''

from numpy import random

from mathclasses import Matrix



class KalmanGeneric(object):
    '''
    classdocs
    '''
    
    #estimation
    x_hat_k = None
    P_hat_k = None

    #update
    x_k = None
    P_k = None
    
    #Matrix
    H = None
    F = None
    B = None
    #G = None
    
    Q = None
    R = None
    
    #z_k = None
    #u_k = None
    #v_k = None
    #w_k = None

    #Estimated state
    #x_est = None
    #P_est = None

    #Updated state
    #x_upd = None
    #P_upd = None

    #Matrices (constants)
    #F = None
    #Q = None
    #H = None
    #R = None
    #G = None

    #Transposed matrices
    #FT = None
    #HT = None
    
    Inputs = 0
    Outputs = 0
    Controls = 0

    def __init__(self, I, O, C):
        '''
        Constructor
        '''
        self.Inputs = I
        self.Outputs = O
        self.Controls = C

        self.x_hat_k = Matrix((O,1))
        self.P_hat_k = Matrix((O,O))
    
        #update
        self.x_k = Matrix((O,1))
        self.P_k = Matrix((O,O))
        
        #Matrix
        self.H = Matrix((I,O))
        self.F = Matrix((O,O))
        self.B = Matrix((O,C))
        #self.G = Matrix((O,C))
        
        self.Q = Matrix((O,O))
        self.R = Matrix((I,I))
        
        '''self.x_k = Matrix([O,1])
        self.P_k = Matrix([O,O])
        self.z_k = Matrix([I,1])
        self.u_k = Matrix([O,1])
        self.v_k = Matrix([I,1])
        self.w_k = Matrix([O,1])
        
        self.x_est = Matrix([O,1])
        self.P_est = Matrix([O,O])
        
        self.x_upd = Matrix([O,1])
        self.P_upd = Matrix([O,O])
        
        self.F = Matrix([O,O])
        self.Q = Matrix([O,O])
        self.H = Matrix([I,O])
        self.R = Matrix([I,I])
        self.G = Matrix([I,1])
        
        self.FT = Matrix([O,O])
        self.HT = Matrix([O,I])'''

    def setMatF(self, arr):
        if len(arr)==self.Outputs and len(arr[0])==self.Outputs:
            self.F = Matrix(arr)
        else:
            print "Bad array size %sx%s for F matrix (expect %sx%s)" % (len(arr), len(arr[0]), self.Outputs, self.Outputs)
    
    def setMatH(self, arr):
        if len(arr)==self.Inputs and len(arr[0])==self.Outputs:
            self.H = Matrix(arr)
        else:
            print "Bad array size %sx%s for H matrix (expect %sx%s)" % (len(arr), len(arr[0]), self.Inputs, self.Outputs)
        
    #def setMatG(self, arr):
    #    if len(arr)==self.Ouputs and len(arr[0]==self.Controls):
    #        self.G = Matrix(arr)
    #    else:
    #        print "Bad array size %sx%s for G matrix" % (len(arr), len(arr[0]))
        
    def newMeasure(self, m, control):
        self.predict(Matrix(control))
        self.update(Matrix(m))
        return self.x_k
    
    def setQ(self, arr):
        if len(arr)==self.Outputs and len(arr[0])==self.Outputs:
            self.Q = Matrix(arr)
        else:
            print "Bad array size %sx%s for Q matrix (expect %sx%s)" % (len(arr), len(arr[0]), self.Outputs, self.Outputs)

    def setB(self, arr):
        if len(arr)==self.Outputs and len(arr[0])==self.Controls:
            self.B = Matrix(arr)
        else:
            print "Bad array size %sx%s for B matrix (expect %sx%s)" % (len(arr), len(arr[0]), self.Outputs, self.Controls)
    
    def setR(self, arr):
        if len(arr)==self.Inputs and len(arr[0])==self.Inputs:
            self.R = Matrix(arr)
        else:
            print "Bad array size %sx%s for R matrix (expect %sx%s)" % (len(arr), len(arr[0]), self.Inputs, self.Inputs)
    
    def setX0(self, arr):
        if len(arr)==self.Outputs and len(arr[0])==1:
            self.x_k = Matrix(arr)
        else:
            print "Bad array size %sx%s for x_k matrix (expect %sx%s)" % (len(arr), len(arr[0]), self.Outputs, 1)
            
    def setP0(self, arr):
        if len(arr)==self.Outputs and len(arr[0])==self.Outputs:
            self.P_k = Matrix(arr)
        else:
            print "Bad array size %sx%s for P_k matrix (expect %sx%s)" % (len(arr), len(arr[0]), self.Outputs, self.Outputs)

    def __repr__(self):
        ret = ""
        ret += "F: \n" + str(self.F)
        ret += "B: \n" + str(self.B)
        ret += "H: \n" + str(self.H)
        
        ret += "Q: \n" + str(self.Q)
        ret += "R: \n" + str(self.R)
        
        return ret
        
    def predict(self, u_k):
        '''
        Predict
        '''
        
        random.seed()
        w_k = Matrix([random.multivariate_normal([0]*self.Outputs, self.Q.components).tolist()]).transpose()
        
        #x_{k|k-1} = F*x_{k-1|k-1} + B*u_k + w_k
        if(self.Controls>0):
            cpart = self.B*u_k
        else:
            cpart = Matrix((self.Outputs, 1))
        
        print cpart
        self.x_hat_k = self.F*self.x_k + cpart + w_k

        #P_{k|k-1} = F*P_{k-1|k-1}*F^T + Q_k
        self.P_hat_k = self.F*self.P_k*self.F.transpose() + self.Q
    
    def update(self, z_k):
        '''
        Update
        '''
        #y_k = z_k - H*x_{k|k-1}
        y = z_k - self.H*self.x_hat_k

        #S_k = H*P_{k|k-1}*H^T + R_k
        S = self.H*self.P_hat_k*self.H.transpose() + self.R

        #K_k = P_{k|k-1}*H^T*S^{-1}
        K = self.P_hat_k*self.H.transpose()*S.invert()
        
        #x_{k|k} = x_{k|k-1} + K_k*y_k
        self.x_k = self.x_hat_k + K*y

        #P_{k|k} = (I-K_k*H_k)*P_{k|k-1}
        self.P_k = (Matrix((self.Outputs, self.Outputs)).identity() - K*self.H)*self.P_hat_k
