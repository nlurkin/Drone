from math import atan2
from numpy.ma.core import cos, sin, arcsin, arctan

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
            self.x = q.x
            self.y = q.y
            self.z = q.z
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
        q.x = self.w*other.x + self.x*other.w + self.y*other.z - self.z*other.y
        q.y = self.w*other.y - self.x*other.z + self.y*other.w + self.z*other.x
        q.z = self.w*other.z + self.x*other.y - self.y*other.x + self.z*other.w
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
        q = Quaternion()
        m = self.mag()
        q.w = self.w/m
        q.x = self.x/m
        q.y = self.y/m
        q.z = self.z/m
    
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
            self.components = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
            self.size = [3,3]
        elif v.__class__ == Matrix:
            self.components = v.components
            self.size = v.size
        elif type(v) == list:
            self.components = []
            for col in v:
                c = []
                for row in col:
                    c.append(row)
                self.components.append(c)
        
    def __mul__(self, other):
        if other.__class__ == Vector:
            v = Vector()
            i = 0
            for col in self.components:
                for row in col:
                    v[i] += row*other[i]
                i+=1
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
                 
if __name__ == "__main__":
    print '*** running test ***'
    q = Quaternion([1,0,0,0])
    v = Vector([1, 2, 3])
    print q
    print v
    print q==q
