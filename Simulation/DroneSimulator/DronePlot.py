import pyqtgraph as pg
from pyqtgraph.Qt import QtCore
from pyqtgraph.functions import mkPen

'''
Created on 12 Mar 2014

@author: Nicoas
'''

class DronePlot(object):
    '''
    classdocs
    '''
    win = pg.GraphicsWindow(title="Drone parameters")
    #win2 = pg.GraphicsWindow(title="Control following")
    
    pQuat = None
    pQuatCurves = [None,None,None,None,None,None]
    pQuatData = [[],[],[],[],[],[]]
    
    pTheta = None
    pThetaCurves = [None,None,None]
    pThetaData = [[],[],[]]
    
    pthetaDot = None
    pThetaDotCurves = [None,None,None]
    pThetaDotData = [[],[],[]]

    pTorque = None
    pTorqueCurves = [None,None,None,None,None,None]
    pTorqueData = [[],[],[],[],[],[]]

    pX= None
    pXCurves = [None,None,None]
    pXData = [[],[],[]]

    pXDot = None
    pXDotCurves = [None,None,None,None,None,None]
    pXDotData = [[],[],[],[],[],[]]
    
    pA = None
    pACurves = [None,None,None]
    pAData = [[],[],[]]
    
    pMotor = None
    pMotorCurves = [None,None,None,None]
    pMotorData = [[],[],[],[]]
        
    pMoveX = None
    pMoveXCruves = [None,None]
    pMoveXData = [[],[]]
    
    pMoveY = None
    pMoveYCruves = [None,None]
    pMoveYData = [[],[]]

    pMoveZ = None
    pMoveZCruves = [None,None]
    pMoveZData = [[],[]]

    pTimes = []
    

    def __init__(self):
        '''
        Constructor
        '''
        self.win.resize(1400,800)
        self.win.setWindowTitle('Attitude plots')
        pg.setConfigOptions(antialias=True)

        #self.win2.resize(1000,600)
        #self.win2.setWindowTitle('Following control')
        
        self.buildQuaternion()
        self.buildTheta()
        self.buildX()
        self.buildXDot()
        self.win.nextRow()
        self.buildThetaDot()
        self.buildTorque()
        self.buildA()
        self.buildMotor()
        #self.buildMoveX()
        #self.buildMoveY()
        #self.buildMoveZ()
        timer = QtCore.QTimer()
        timer.timeout.connect(self.update)
        timer.start(50)

        
    def buildQuaternion(self):
        self.pQuat = self.win.addPlot(title="Quaternion")
        self.pQuat.showGrid(x=True,y=True)
        self.pQuatCurves[0] = self.pQuat.plot(pen="r")
        self.pQuatCurves[1] = self.pQuat.plot(pen="g")
        self.pQuatCurves[2] = self.pQuat.plot(pen="b")
        self.pQuatCurves[3] = self.pQuat.plot(pen=mkPen(color="r",style=QtCore.Qt.DotLine))
        self.pQuatCurves[4] = self.pQuat.plot(pen=mkPen(color="b",style=QtCore.Qt.DotLine))
        self.pQuatCurves[5] = self.pQuat.plot(pen=mkPen(color="g",style=QtCore.Qt.DotLine))
    
    def addQuaternion(self, q, qref):
        self.pQuatData[0].append(q.x)
        self.pQuatData[1].append(q.y)
        self.pQuatData[2].append(q.z)
        self.pQuatData[3].append(qref.x)
        self.pQuatData[4].append(qref.y)
        self.pQuatData[5].append(qref.z)
    
    def updateQuaternion(self):
        self.pQuatCurves[0].setData(self.pTimes, self.pQuatData[0])
        self.pQuatCurves[1].setData(self.pTimes, self.pQuatData[1])
        self.pQuatCurves[2].setData(self.pTimes, self.pQuatData[2])
        self.pQuatCurves[3].setData(self.pTimes, self.pQuatData[3])
        self.pQuatCurves[4].setData(self.pTimes, self.pQuatData[4])
        self.pQuatCurves[5].setData(self.pTimes, self.pQuatData[5])


    def buildTheta(self):
        self.pTheta = self.win.addPlot(title="Theta")
        self.pTheta.showGrid(x=True,y=True)
        self.pThetaCurves[0] = self.pTheta.plot(pen="r")
        self.pThetaCurves[1] = self.pTheta.plot(pen="b")
        self.pThetaCurves[2] = self.pTheta.plot(pen="g")
    
    def addTheta(self, theta):
        self.pThetaData[0].append(theta[0])
        self.pThetaData[1].append(theta[1])
        self.pThetaData[2].append(theta[2])

    def updateTheta(self):
        self.pThetaCurves[0].setData(self.pTimes, self.pThetaData[0])
        self.pThetaCurves[1].setData(self.pTimes, self.pThetaData[1])
        self.pThetaCurves[2].setData(self.pTimes, self.pThetaData[2])
        
    def buildThetaDot(self):
        self.pThetaDot = self.win.addPlot(title="Theta dot")
        self.pThetaDot.showGrid(x=True,y=True)
        self.pThetaDotCurves[0] = self.pThetaDot.plot(pen="r")
        self.pThetaDotCurves[1] = self.pThetaDot.plot(pen="b")
        self.pThetaDotCurves[2] = self.pThetaDot.plot(pen="g")
    
    def addThetaDot(self, thetad):
        self.pThetaDotData[0].append(thetad[0])
        self.pThetaDotData[1].append(thetad[1])
        self.pThetaDotData[2].append(thetad[2])
        
    def updateThetaDot(self):
        self.pThetaDotCurves[0].setData(self.pTimes, self.pThetaDotData[0])
        self.pThetaDotCurves[1].setData(self.pTimes, self.pThetaDotData[1])
        self.pThetaDotCurves[2].setData(self.pTimes, self.pThetaDotData[2])

    def buildTorque(self):
        self.pTorque = self.win.addPlot(title="Torque")
        self.pTorque.showGrid(x=True,y=True)
        self.pTorqueCurves[0] = self.pTorque.plot(pen="r")
        self.pTorqueCurves[1] = self.pTorque.plot(pen="b")
        self.pTorqueCurves[2] = self.pTorque.plot(pen="g")
        self.pTorqueCurves[3] = self.pTorque.plot(pen=mkPen(color="r",style=QtCore.Qt.DotLine))
        self.pTorqueCurves[4] = self.pTorque.plot(pen=mkPen(color="b",style=QtCore.Qt.DotLine))
        self.pTorqueCurves[5] = self.pTorque.plot(pen=mkPen(color="g",style=QtCore.Qt.DotLine))
    
    def addTorque(self, tau, tauref):
        self.pTorqueData[0].append(tau[0])
        self.pTorqueData[1].append(tau[1])
        self.pTorqueData[2].append(tau[2])
        self.pTorqueData[3].append(tauref[0])
        self.pTorqueData[4].append(tauref[1])
        self.pTorqueData[5].append(tauref[2])
    
    def updateTorque(self):
        self.pTorqueCurves[0].setData(self.pTimes, self.pTorqueData[0])
        self.pTorqueCurves[1].setData(self.pTimes, self.pTorqueData[1])
        self.pTorqueCurves[2].setData(self.pTimes, self.pTorqueData[2])
        self.pTorqueCurves[3].setData(self.pTimes, self.pTorqueData[3])
        self.pTorqueCurves[4].setData(self.pTimes, self.pTorqueData[4])
        self.pTorqueCurves[5].setData(self.pTimes, self.pTorqueData[5])

    def buildX(self):
        self.pX= self.win.addPlot(title="X")
        self.pX.showGrid(x=True,y=True)
        self.pXCurves[0] = self.pX.plot(pen="r")
        self.pXCurves[1] = self.pX.plot(pen="b")
        self.pXCurves[2] = self.pX.plot(pen="g")
    
    def addX(self, x):
        self.pXData[0].append(x[0])
        self.pXData[1].append(x[1])
        self.pXData[2].append(x[2])
        
    def updateX(self):
        self.pXCurves[0].setData(self.pTimes, self.pXData[0])
        self.pXCurves[1].setData(self.pTimes, self.pXData[1])
        self.pXCurves[2].setData(self.pTimes, self.pXData[2])
            
    def buildXDot(self):
        self.pXDot =    self.win.addPlot(title="X Dot")
        self.pXDot.showGrid(x=True,y=True)
        self.pXDotCurves[0] = self.pXDot.plot(pen="r")
        self.pXDotCurves[1] = self.pXDot.plot(pen="b")
        self.pXDotCurves[2] = self.pXDot.plot(pen="g")
        self.pXDotCurves[3] = self.pXDot.plot(pen=mkPen(color="r",style=QtCore.Qt.DotLine))
        self.pXDotCurves[4] = self.pXDot.plot(pen=mkPen(color="b",style=QtCore.Qt.DotLine))
        self.pXDotCurves[5] = self.pXDot.plot(pen=mkPen(color="g",style=QtCore.Qt.DotLine))
    
    def addXDot(self, xdot, xdotref):
        self.pXDotData[0].append(xdot[0])
        self.pXDotData[1].append(xdot[1])
        self.pXDotData[2].append(xdot[2])
        self.pXDotData[3].append(xdotref[0])
        self.pXDotData[4].append(xdotref[1])
        self.pXDotData[5].append(xdotref[2])
    
    def updateXDot(self):
        self.pXDotCurves[0].setData(self.pTimes, self.pXDotData[0])
        self.pXDotCurves[1].setData(self.pTimes, self.pXDotData[1])
        self.pXDotCurves[2].setData(self.pTimes, self.pXDotData[2])
        self.pXDotCurves[3].setData(self.pTimes, self.pXDotData[3])
        self.pXDotCurves[4].setData(self.pTimes, self.pXDotData[4])
        self.pXDotCurves[5].setData(self.pTimes, self.pXDotData[5])

    def buildA(self):
        self.pA= self.win.addPlot(title="Acceleration")
        self.pA.showGrid(x=True,y=True)
        self.pACurves[0] = self.pA.plot(pen="r")
        self.pACurves[1] = self.pA.plot(pen="b")
        self.pACurves[2] = self.pA.plot(pen="g")
    
    def addA(self, a):
        self.pAData[0].append(a[0])
        self.pAData[1].append(a[1])
        self.pAData[2].append(a[2])
        
    def updateA(self):
        self.pACurves[0].setData(self.pTimes, self.pAData[0])
        self.pACurves[1].setData(self.pTimes, self.pAData[1])
        self.pACurves[2].setData(self.pTimes, self.pAData[2])

    def buildMotor(self):
        self.pMotor= self.win.addPlot(title="Motor")
        self.pMotor.showGrid(x=True,y=True)
        self.pMotorCurves[0] = self.pMotor.plot(pen="r")
        self.pMotorCurves[1] = self.pMotor.plot(pen="b")
        self.pMotorCurves[2] = self.pMotor.plot(pen="g")
        self.pMotorCurves[3] = self.pMotor.plot(pen="y")
    
    def addMotor(self, omega):
        self.pMotorData[0].append(omega[0])
        self.pMotorData[1].append(omega[1])
        self.pMotorData[2].append(omega[2])
        self.pMotorData[3].append(omega[3])
        
    def updateMotor(self):
        self.pMotorCurves[0].setData(self.pTimes, self.pMotorData[0])
        self.pMotorCurves[1].setData(self.pTimes, self.pMotorData[1])
        self.pMotorCurves[2].setData(self.pTimes, self.pMotorData[2])
        self.pMotorCurves[3].setData(self.pTimes, self.pMotorData[3])
    
    def buildMoveX(self):
        self.pMoveX= self.win2.addPlot(title="Move X")
        self.pMoveX.showGrid(x=True,y=True)
        self.pMoveXCurves[0] = self.pMoveX.plot(pen="r")
        self.pMoveXCurves[1] = self.pMoveX.plot(pen=mkPen(color="r",style=QtCore.Qt.DotLine))
    
    def addMoveX(self, x, xref):
        self.pMoveXData[0].append(x)
        self.pMovexData[1].append(xref)
        
    def updateMoveX(self):
        self.pMoveXCurves[0].setData(self.pTimes, self.pMoveXData[0])
        self.pMoveXCurves[1].setData(self.pTimes, self.pMoveXData[1])

    def buildMoveY(self):
        self.pMoveY= self.win2.addPlot(title="Move Y")
        self.pMoveY.showGrid(x=True,y=True)
        self.pMoveYCurves[0] = self.pMoveY.plot(pen="g")
        self.pMoveYCurves[1] = self.pMoveY.plot(pen=mkPen(color="g",style=QtCore.Qt.DotLine))
    
    def addMoveY(self, y, yref):
        self.pMoveYData[0].append(y)
        self.pMoveYData[1].append(yref)
        
    def updateMoveY(self):
        self.pMoveYCurves[0].setData(self.pTimes, self.pMoveYData[0])
        self.pMoveYCurves[1].setData(self.pTimes, self.pMoveYData[1])

    def buildMoveZ(self):
        self.pMoveZ= self.win2.addPlot(title="Move Z")
        self.pMoveZ.showGrid(x=True,y=True)
        self.pMoveZCurves[0] = self.pMoveZ.plot(pen="b")
        self.pMoveZCurves[1] = self.pMoveZ.plot(pen=mkPen(color="b",style=QtCore.Qt.DotLine))
    
    def addMoveZ(self, z, zref):
        self.pMoveZData[0].append(z)
        self.pMoveZData[1].append(zref)
        
    def updateMoveZ(self):
        self.pMoveZCurves[0].setData(self.pTimes, self.pMoveZData[0])
        self.pMoveZCurves[1].setData(self.pTimes, self.pMoveZData[1])

    def addTime(self,t):
        self.pTimes.append(t)        
    
    def update(self):
        self.updateQuaternion()
        self.updateTheta()
        self.updateThetaDot()
        self.updateTorque()
        self.updateX()
        self.updateXDot()
        self.updateA()
        self.updateMotor()
        #self.updateMoveX()
        #self.updateMoveY()
        #self.updateMoveY()
        