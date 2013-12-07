from numpy.ma.core import sin, cos, tan
def vecCrossProduct(a, b): #vector, vector 
    c = [[a[1]*b[2]-a[2]*b[1]], [b[2]*a[0]-a[2]*b[0]], [a[0]*b[1]-b[0]*a[1]]]
    return c

def vecDotProduct(a, b): #vector, vector
    c = [a[0]*b[0] + a[1]*b[1] + a[2]*b[2]]
    return c

def matDotProduct(a, b): #matrix, vector
    c = [vecDotProduct(a[0], b)[0], vecDotProduct(a[1], b)[0], vecDotProduct(a[2], b)[0]]
    return c

def vecSum(a, b): #vector, vector
    c = [a[0]+b[0], a[1]+b[1], a[2]+b[2]]
    return c

def vecDiff(a, b): #vector, vector
    return vecSum(a, vecScalarProduct(-1, b))
    
def vecScalarProduct(a, b): #scalar, vector
    c = [a*b[0], a*b[1], a*b[2]]
    return c

def vecScalarSum(a, b): #scalar, vector
    c = [a+b[0], a+b[1], a+b[2]]
    return c


def thetadot2omega(tD, t):
    mat1 = [[1, 0, -sin(t[1])], [0, cos(t[0]), cos(t[1])*sin(t[0])], [0, -sin(t[0]), cos(t[1])*cos(t[0])]]
    #mat2 = [[0, -sin(t[0]), cos(t[0])*sin(t[1])], [0, cos(t[0]), sin(t[0])*cos(t[1])], [1, 0, -sin(t[1])]]
    
    return matDotProduct(mat1, tD)

def rotation(t):
    mat = [[cos(t[0])*sin(t[2])-cos(t[1])*sin(t[0])*sin(t[2]), -cos(t[2])*sin(t[0])-cos(t[0])*cos(t[1])*sin(t[2]), sin(t[1])*sin(t[2])], 
           [cos(t[1])*cos(t[2])*sin(t[0])+cos(t[0])*sin(t[2]), cos(t[0])*cos(t[1])*cos(t[2])-sin(t[0])*sin(t[2]), -cos(t[2])*sin(t[1])], 
           [sin(t[0])*sin(t[1]), cos(t[0])*sin(t[1]), cos(t[1])]]
    return mat

def omega2thetadot(t, o):
    mat1 = [[1, sin(t[0])*tan(t[1]), cos(t[0])*tan(t[1])], 
            [0, cos(t[0]), -sin(t[0])], 
            [0, sin(t[0])/cos(t[1]), cos(t[0])/cos(t[1])]]
    #mat2 = [[0, -sin(t[0]), cos(t[0])*sin(t[1])], [0, cos(t[0]), sin(t[0])*cos(t[1])], [1, 0, -sin(t[1])]]
    
    return matDotProduct(mat1, o)

def acceleration(angles, b):
    gravity = [0,0,-9.81]
    R = rotation(angles)
    T = matDotProduct(R, b.thrust())
    Fd = b.friction()
    a = vecSum(gravity, vecSum(vecScalarProduct(1/b.mass(), T), Fd))
    return a
