class Quaternion:
    '''
    classdocs
    '''
    
    w = None
    x = None
    y = None
    z = None

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
                '''From euler angles
                data[0] = atan2(2*q -> x*q -> y - 2*q -> w*q -> z, 2*q -> w*q -> w + 2*q -> x*q -> x - 1);
                // pitch: (nose up/down, about Y axis)
                data[1] = atan(gravity -> x / sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
                // roll: (tilt left/right, about X axis)
                data[2] = atan(gravity -> y / sqrt(gravity -> x*gravity -> x + gravity -> z*gravity -> z));
                '''
        
        def __str__(self):
            return '(' + self.w + ',' + self.x + ',' + self.y + ',' + self.z + ')'
        
        def __add__(self, other):
            q = Quaternion
            q.w = self.w + other.w
            q.x = self.x + other.x
            q.y = self.y + other.y
            q.z = self.z + other.z
            return q
    
        def __sub__(self, other):
            q = Quaternion
            q.w = self.w - other.w
            q.x = self.x - other.x
            q.y = self.y - other.y
            q.z = self.z - other.z
            return q
        
        def __mul__(self, other):
            q = Quaternion
            q.w = self.w*other.w - self.x*other.x - self.y*other.y - self.z*other.z
            q.x = self.w*other.x + self.x*other.w + self.y*other.z - self.z*other.y
            q.y = self.w*other.y - self.x*other.z + self.y*other.w + self.z*other.x
            q.z = self.w*other.z + self.x*other.y - self.y*other.x + self.z*other.w
            return q
        
        def __neg__(self):
            q = Quaternion
            q.w = -self.w
            q.x = -self.x
            q.y = -self.y
            q.z = -self.z
            return q
        
        def conj(self):
            q = Quaternion
            q.w = self.w
            q.x = -self.w
            q.y = -self.y
            q.z = -self.z
            return q
        
        def mag(self):
            return (self.w*self.w + self.x*self.x + self.y*self.y + self.z*self.z)
        
        def normalize(self):
            q = Quaternion
            m = self.mag()
            q.w = self.w/m
            q.x = self.x/m
            q.y = self.y/m
            q.z = self.z/m
        
        def angles(self):
            a = []
            '''#yaw
            a.append(atan2(2*self.x*self.y - 2*self.w*self.z, 2*self.w*self.w + 2*self.x*self.x - 1))
            #pitch
            a.append(atan(gravity -> x / sqrt(gravity -> y*gravity -> y + gravity -> z*gravity -> z));
            #roll: (tilt left/right, about X axis)
            data[2] = atan(gravity -> y / sqrt(gravity -> x*gravity -> x + gravity -> z*gravity -> z));
            '''
            return a
        
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
            return ((self.w==other.w) and (self.x==other.x) and (self.y==other.y) and (self.z==other.z))

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
        elif v.__class__==Quaternion:
            self.components = v[1:3]
        elif type(v)==list:
            for el in v:
                self.append(el)
        else:
            print "Unable to create Vector from "
            print v
    
    def append(self, el):
        self.components.append(el)
        self.size += 1
    
    def __getitem(self, key):
        return self.components[key]
    
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
        v = Vector
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
            v = Vector
            i = 0
            for el in self.components:
                v.append(el+other[i])
                i+=1
            return v
        else:
            v = Vector
            for el in self.components:
                v.append(el+other)
            return v
    
    def __sub__(self, other):
        return self.__add__(-other)
        
    def __mul__(self, other):
        if other.__class__ == Vector and other.size==self.size:
            val = 0
            i = 0
            for el in self.components:
                val += el*other[i]
                i+=1
            return val
        else:
            v = Vector
            for el in self.components:
                v.append(el*other)
            return v
    
    def __div__(self, other):
        print "xxx"
    
    def __neg__(self):
        v = Vector
        for el in self.components:
            v.append(-el)
        
        return v
    
    def __eq(self, other):
        val = True
        i=0
        if self.size!=other.size:
            return False
        
        for el in self.components:
            val = (val and el==other[i])
        return val

if __name__ == "__main__":
    print '*** running test ***'
    import mathclasses
    q = Quaternion([1,0,0,0])
    v = Vector([1, 2, 3])
    print q
    print v
    print q==q
