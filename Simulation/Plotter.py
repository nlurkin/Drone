'''
Created on 15 Nov 2014

@author: Nicoas
'''
import string
import sys

from pyqtgraph.Qt import QtGui, QtCore
from pyqtgraph.functions import mkPen
import serial

from ParamsClass import Params
import pyqtgraph as pg


class Plotter(object):
    '''
    classdocs
    '''


    def __init__(self):
        '''
        Constructor
        '''
        self.win = pg.GraphicsWindow(title="Drone parameters")
        
        self.win.resize(1400,800)
        self.win.setWindowTitle('Attitude plots')
        pg.setConfigOptions(antialias=True)
        
        self.pOmega = None
        self.pOmegaCurves = [None,None]
        self.pOmegaData = [[],[]]

        self.pAlpha = None
        self.pAlphaCurves = [None,None]
        self.pAlphaData = [[],[]]

        self.pDelta = None
        self.pDeltaCurves = [None]
        self.pDeltaData = [[]]

        self.pTimes = []
        
        self.buildOmega()
        self.buildAlpha()
        self.buildDelta()
        
        
    def addData(self, t, omega, alpha, oOmega, oAlpha, oDelta):
        self.addOmega(float(omega), float(oOmega))
        self.addAlpha(float(alpha), float(oAlpha))
        self.addDelta(float(oDelta))
        self.pTimes.append(float(t))
        
    def buildOmega(self):
        self.pOmega = self.win.addPlot(title="Omega", row=0, col=0)
        self.pOmega.showGrid(x=True,y=True)
        self.pOmegaCurves[0] = self.pOmega.plot(pen="g")
        self.pOmegaCurves[1] = self.pOmega.plot(pen=mkPen(color="r",style=QtCore.Qt.DotLine))

    def addOmega(self, omega, oOmega):
        self.pOmegaData[0].append(omega)
        self.pOmegaData[1].append(oOmega)
        
    def updateOmega(self):
        self.pOmegaCurves[0].setData(self.pTimes, self.pOmegaData[0])
        self.pOmegaCurves[1].setData(self.pTimes, self.pOmegaData[1])

    def buildAlpha(self):
        self.pAlpha = self.win.addPlot(title="Alpha", row=1, col=0)
        self.pAlpha.showGrid(x=True,y=True)
        self.pAlphaCurves[0] = self.pAlpha.plot(pen="g")
        self.pAlphaCurves[1] = self.pAlpha.plot(pen=mkPen(color="r",style=QtCore.Qt.DotLine))

    def addAlpha(self, alpha, oAlpha):
        self.pAlphaData[0].append(alpha)
        self.pAlphaData[1].append(oAlpha)

    def updateAlpha(self):
        self.pAlphaCurves[0].setData(self.pTimes, self.pAlphaData[0])
        self.pAlphaCurves[1].setData(self.pTimes, self.pAlphaData[1])
        
    def buildDelta(self):
        self.pDelta = self.win.addPlot(title="Delta", row=2, col=0)
        self.pDelta.showGrid(x=True,y=True)
        self.pDeltaCurves[0] = self.pDelta.plot(pen="g")
            
    def addDelta(self, delta):
        self.pDeltaData[0].append(delta)
    
    def updateDelta(self):
        self.pDeltaCurves[0].setData(self.pTimes, self.pDeltaData[0])
    
    def update(self):
        self.updateOmega()
        self.updateAlpha()
        self.updateDelta()


def serialLoop():
    s = str(ser.readline());
    
    if len(s) > 0:
        s = string.rstrip(s, "\r\n")
        print s
        return s.split(" ")
    
if __name__=="__main__":
    ser = serial.Serial(port=Params.comPort, baudrate=9600, timeout=1)
    p = Plotter()
    
    while(True):
        QtGui.QApplication.processEvents()
        arr = serialLoop()
        if len(arr)==7:
            p.addData(arr[0], arr[2], arr[1], arr[4], arr[5], float(arr[1])-float(arr[5]))
            p.update()
    
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()

