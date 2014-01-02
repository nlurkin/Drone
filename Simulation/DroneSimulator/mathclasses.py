from math import atan2
from numpy.lib.scimath import sqrt
from numpy.ma.core import cos, sin, arcsin, arctan

def createRotation(origin, dest):
    q = Quaternion(origin.cross(dest))
    q.w = sqrt(origin.mag() * dest.mag()) + origin*dest;
    q.normalize()
    return q

class Quaternion:
    '''
    classdocs
    '''
    
    w = 0
    x = 0
    y = 0
    z = 0

    def __init__(self, q=None):
        '''
        Constructor
        '''
        if q==None:
            self.w = 1.0
            self.x = 0.
            self.y = 0.
            self.z = 0.
        elif q.__class__==Quaternion:
            self.w = q.w
            self.x = q.x
            self.y = q.y
            self.z = q.z
        elif q.__class__==Vector:
            self.w = 0
            self.x = q[0]
            self.y = q[1]
            self.z = q[2]
            #self.normalize()
        elif type(q)==list:
            if len(q)==4:
                self.w = q[0]
                self.x = q[1]
                self.y = q[2]
                self.z = q[3]
            elif len(q)==3:
                q[0] = float(q[0])
                q[1] = float(q[1])
                q[2] = float(q[2])
                self.w = cos(q[0]/2)*cos(q[1]/2)*cos(q[2]/2)+sin(q[0]/2)*sin(q[1]/2)*sin(q[2]/2)
                self.x = sin(q[0]/2)*cos(q[1]/2)*cos(q[2]/2)-cos(q[0]/2)*sin(q[1]/2)*sin(q[2]/2)
                self.y = cos(q[0]/2)*sin(q[1]/2)*cos(q[2]/2)+sin(q[0]/2)*cos(q[1]/2)*sin(q[2]/2)
                self.z = cos(q[0]/2)*cos(q[1]/2)*sin(q[2]/2)-sin(q[0]/2)*sin(q[1]/2)*cos(q[2]/2)
        #self.normalize()
        
        
    def __str__(self):
        return '(' + str(self.w) + ',' + str(self.x) + ',' + str(self.y) + ',' + str(self.z) + ')'
    
    def __repr__(self):
        return self.__str__()
    
    def __add__(self, other):
        q = Quaternion()
        q.w = self.w + other.w
        q.x = self.x + other.x
        q.y = self.y + other.y
        q.z = self.z + other.z
        q.normalize()
        return q
    
    def __radd__(self, other):
        if type(other)==str:
            return other + self.__str__()

    def __sub__(self, other):
        q = Quaternion()
        q.w = self.w - other.w
        q.x = self.x - other.x
        q.y = self.y - other.y
        q.z = self.z - other.z
        return q
    
    def __mul__(self, other):
        q = Quaternion()
        q.w = self.w*other.w - self.x*other.x - self.y*other.y - self.z*other.z
        q.x = self.w*other.x + self.x*other.w - self.y*other.z + self.z*other.y
        q.y = self.w*other.y + self.x*other.z + self.y*other.w - self.z*other.x
        q.z = self.w*other.z - self.x*other.y + self.y*other.x + self.z*other.w
        return q
    
    def __neg__(self):
        q = Quaternion()
        q.w = -self.w
        q.x = -self.x
        q.y = -self.y
        q.z = -self.z
        return q
    
    def conj(self):
        q = Quaternion()
        q.w = self.w
        q.x = -self.x
        q.y = -self.y
        q.z = -self.z
        return q
    
    def mag(self):
        return (self.w*self.w + self.x*self.x + self.y*self.y + self.z*self.z)
    
    def normalize(self):
        m = sqrt(self.mag())
        self.w = self.w/m
        self.x = self.x/m
        self.y = self.y/m
        self.z = self.z/m
    
    def angles(self):
        a = [0, 0, 0]
        a[0] = arctan(2*(self.w*self.x + self.y*self.z)/(1-2*(self.x*self.x + self.y*self.y)))
        a[1] = arcsin(2*(self.w*self.y - self.z*self.x))
        a[2] = arctan(2*(self.w*self.z + self.x*self.y)/(1-2*(self.y*self.y + self.z*self.z)))
        return Vector(a)
    
    def vector(self):
        return Vector([self.x, self.y, self.z])
        
    def __getitem__(self, key):
        if key==0:
            return self.w
        if key==1:
            return self.x
        if key==2:
            return self.y
        if key==3:
            return self.z
    
    def __eq__(self, other):
        if other.__class__==Quaternion:
            return ((self.w==other.w) and (self.x==other.x) and (self.y==other.y) and (self.z==other.z))
        else:
            return False

class Vector:
    '''
    classdocs
    '''

    components = []
    size = 0

    def __init__(self, v=None):
        '''
        Constructor
        '''
        if v==None:
            self.components = [0, 0, 0]
            self.size = 3
        elif v.__class__ == Vector:
            self.components = v.components
            self.size = v.size
        elif v.__class__==Quaternion:
            self.components = [v[1], v[2], v[3]]
            self.size = 3
        elif type(v)==list:
            self.components = []
            for el in v:
                self.append(el)
        else:
            print "Unable to create Vector from " + type(v)
    
    def append(self, el):
        self.components.append(el)
        self.size += 1
    
    def __getitem__(self, key):
        if len(self.components)>key:
            return self.components[key]
        else:
            return None
    
    def __setitem__(self, key, value):
        while len(self.components)<key:
            self.components.append(0)
        self.components[key] = value
    
    def __str__(self):
        s = ''
        for el in self.components:
            s += str(el)
            s += ','
        s = s[:-1]
        
        return '(' + s + ')' 
    
    def mag(self):
        val = 0
        for el in self.components:
            val += el*el
        return val
    
    def normalize(self):
        m = self.mag()
        v = Vector()
        for el in self.components:
            v.append(el/m)
        
        return v
    
    def rotate(self, q):
        p = Quaternion(self)
        p = q*p
        p = p*q.conj()
        return p.vector()
    
    def __add__(self, other):
        if other.__class__ == Vector and other.size==self.size:
            v = []
            i = 0
            for el in self.components:
                v.append(el+other[i])
                i+=1
            return Vector(v)
        else:
            v = []
            for el in self.components:
                v.append(el+other)
            return Vector(v)
    
    def __radd__(self, other):
        if type(other) == str:
            return other + self.__str__()
        else:
            return self.__add__(other)
    
    def __sub__(self, other):
        return self.__add__(-other)
    
    def __rsub__(self, other):
        return -self.__sub__(other)
        
    def __mul__(self, other):
        if other.__class__ == Vector and other.size==self.size:
            val = 0
            i = 0
            for el in self.components:
                val += el*other[i]
                i+=1
            return val
        else:
            v = []
            for el in self.components:
                v.append(el*other)
            return Vector(v)
    
    def __rmul__(self, other):
        return self.__mul__(other)
    
    def __div__(self, other):
        v = []
        for el in self.components:
            v.append(el/other)
        return Vector(v)
    
    def __neg__(self):
        v = []
        for el in self.components:
            v.append(-el)
        return Vector(v)
    
    def __eq(self, other):
        val = True
        i=0
        if self.size!=other.size:
            return False
        
        for el in self.components:
            val = (val and el==other[i])
        return val
    
    def __repr__(self):
        return self.__str__()

    def cross(self, other):
        v = Vector()
        if self.size==other.size:
            if self.size==3:
                v[0] = self.components[1]*other[2]-self.components[2]*other[1]
                v[1] = self.components[2]*other[0]-self.components[0]*other[2]
                v[2] = self.components[0]*other[1]-self.components[1]*other[0]
        return v
        
class Matrix:
    '''
    classdocs
    '''

    components = []
    size = [0,0]

    def __init__(self, v=None):
        '''
        Constructor
        '''
        if v==None:
            self.components = [[0., 0., 0.], [0., 0., 0.], [0., 0., 0.]]
            self.size = [3,3]
        elif v.__class__ == Matrix:
            self.components = v.components[:]
            self.size = v.size
        elif type(v) == list:
            if len(v)==2 and type(v[0])!=list:
                self.components = []
                for i in range(0,v[0]):
                    row = []
                    for j in range(0,v[1]):
                        row.append(0.)
                    self.components.append(row)
                self.size = v
            else:
                self.components = []
                for col in v:
                    c = []
                    for row in col:
                        c.append(row)
                    self.components.append(c)
                self.size = [len(v),len(v[0])]
        
    def __mul__(self, other):
        if other.__class__ == Vector:
            v = Vector([0.]*self.size[0])
            i = 0
            for col in self.components:
                j=0
                for row in col:
                    v[i] += row*other[j]
                    j+=1
                i+=1
            return v
        elif other.__class__== Matrix:
            v = Matrix([self.size[0],other.size[1]])
            for ir in range(0,self.size[0]):
                for ic in range(0,other.size[1]):
                    sum = 0
                    for a,b in zip(self.row(ir), other.col(ic)):
                        sum += a*b
                    v.setitem(ir,ic,sum)
            return v
        else:
            v = Matrix()
            i=0
            j=0
            for col in self.components:
                for row in col:
                    v[i,j] = row*other
                    j+=1
                i+=1
            return v
    
    def __repr__(self):
        return self.__str__()
    
    def __str__(self):
        s = ''
        for col in self.components:
            for row in col:
                s += str(row)
                s += ', '
            s = s[:-1]
            s += '\n '
        s = s[:-2]
        return '[' + s + ']'
    
    def setrow(self,n,l):
        self.components[n] = l
    
    def setcol(self,n,l):
        for k in range(0,self.size[0]):
            self.components[k][n] = l[k]
    
    def setitem(self,n,m,v):
        self.components[n][m] = v
    
    def col(self,n):
        return [el[n] for el in self.components]
    
    def row(self, n):
        return self.components[n]
    
    def appendrow(self, r):
        self.components.append(r)
        self.size[0]+=1
    
    def maxcol(self,n):
        col = self.col(n)
        abscol = [abs(el) for el in col]
        m = max(abscol)
        i = abscol.index(m)
        return [self.col(n)[i], i]
    
    def swap(self,n,m):
        temp = self.row(n)
        self.setrow(n, self.row(m))
        self.setrow(m,temp)
        
    def swapcol(self,n,m):
        temp = self.col(n)
        self.setcol(n, self.col(m))
        self.setcol(m, temp)
    
    def dividerow(self,n,v):
        self.components[n] = [el/float(v) for el in self.row(n)]
    
    def substractrow(self,n,mult,m):
        self.components[n] = [el1-el2*mult for el1,el2 in zip(self.row(n),self.row(m))]
        
    def __getitem__(self,n):
        return self.components[n]
    
    def identity(self):
        for k in range(0, self.size[0]):
            self.components[k][k] = 1.
    def invert(self):
        pivrow = 0  #keeps track of current pivot row
        pivrows = []  #keeps track of rows swaps to undo at end
        n = self.size[0]
        
        inv = Matrix(self)
        idMat = Matrix(self.size)
        idMat.identity()
        
        for k in range(0,n):
            #find pivot row, the row with biggest entry in current column
            [m, pivrow] = inv.maxcol(k)

            #check for singular matrix
            if m == 0.0:
                print "Inversion failed due to singular matrix"
                return False
            if pivrow != k:
                inv.swap(k, pivrow)
                idMat.swap(k,pivrow)
            
            pivrows.append(pivrow)  #record row swap (even if no swap happened)
            inv.dividerow(k, m)
            idMat.dividerow(k, m)
            #Now eliminate all other entries in this column
            for i in range(0,n):
                if i != k:
                    idMat.substractrow(i, inv[i][k], k)
                    inv.substractrow(i, inv[i][k], k)
            
        return idMat

                 
if __name__ == "__main__":
    print '*** running test ***'
    x = Matrix([[3., 0., 2.],[2.,0.,-2.],[0.,1.,1.]])
    y = x.invert()
    print x*y