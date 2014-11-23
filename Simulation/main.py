#!python
from pyqtgraph.Qt import QtGui, QtCore
from ParamsClass import Params
from DroneSimulator.FullSimu import Simu
from DroneMath.mathclasses import Quaternion, Vector
from time import sleep, time
import msvcrt
import serial
import string
import sys

torqueSet = 0
reqTorque = Vector([0, 0, 0])
reqNextStep = 0
reqData = False

ser = None

simu = Simu()

#console
def readInput( caption, default, timeout = 5):
    start_time = time()
    inputS = ''
    while True:
        if msvcrt.kbhit():
            chrS = msvcrt.getche()
            if ord(chrS) == 13: # enter_key
                break
            elif ord(chrS) >= 32: #space_char
                inputS += chrS
        if len(inputS) == 0 and (time() - start_time) > timeout:
            break

    if len(inputS) > 0:
        return inputS
    else:
        return default

#microcontrolleur
def sendSensor():
    if Params.runLocally==True:
        return
    prefix = "DAT:SENS:"
    quat = simu.b.Quat
    gyro = simu.b.Omega
    acc = simu.b.Acceleration
    t = simu.t
    print "Sending quat " + str(quat)
    print "Sending gyro " + str(gyro)
    print "Sending acc " + str(acc)
    print "With alpha " + str(simu.b.Alpha)
    ser.write(prefix + "BUF0:" + str(quat.w) + "\r\n")
    sleep(Params.serialSleep)
    ser.write(prefix + "BUF1:" + str(quat.x) + "\r\n")
    sleep(Params.serialSleep)
    ser.write(prefix + "BUF2:" + str(quat.y) + "\r\n")
    sleep(Params.serialSleep);
    ser.write(prefix + "BUF3:" + str(quat.z) + "\r\n")
    sleep(Params.serialSleep)
    ser.write(prefix + "BUF4:" + str(int(gyro[0]*131*2)) + "\r\n")
    sleep(Params.serialSleep)
    ser.write(prefix + "BUF5:" + str(int(gyro[1]*131*2)) + "\r\n")
    sleep(Params.serialSleep)
    ser.write(prefix + "BUF6:" + str(int(gyro[2]*131*2)) + "\r\n")
    sleep(Params.serialSleep)
    ser.write(prefix + "BUF7:" + str(int(acc[0]*8192*2)) + "\r\n")
    sleep(Params.serialSleep)
    ser.write(prefix + "BUF8:" + str(int(acc[1]*8192*2)) + "\r\n")
    sleep(Params.serialSleep)
    ser.write(prefix + "BUF9:" + str(int(acc[2]*8192*2)) + "\r\n")
    sleep(Params.serialSleep)
    ser.write(prefix + "TIME:" + str(t) + "\r\n")

def sendTime():
    t = simu.t
    ser.write("DAT:TIME:" + str(t) + "\r\n")      

def sendI():
    I = simu.I
    prefix = "DAT:IMAT:"
    ser.write(prefix + "IXX:" + str(I[0]) + "\r\n")
    ser.write(prefix + "IYY:" + str(I[1]) + "\r\n")
    ser.write(prefix + "IZZ:" + str(I[2]) + "\r\n")
    print "Sending %s %s %s" % (I[0], I[1], I[2])

def sendDebug():
    prefix = "CMD:CTRL:"
    ser.write(prefix + "GODEBUG\r\n")

def sendKValues():
    prefix = "CMD:SETK:"
    ser.write(prefix + "SIMKP:" + str(6) + "\r\n")
    ser.write(prefix + "SIMKD:" + str(30) + "\r\n")
    
def sendNewTracking(anglesSet):
    if anglesSet is None:
        angle= simu.getNextMove()
        v = Vector()
        quat = Quaternion([angle[0], angle[1], angle[2]])
    else:
        quat = Quaternion(anglesSet)
        a = Vector([0,0,0])
        v = Vector([0,0,0])
        #v = Vector()
    simu.setReference(quat,v)
    
    if not Params.runLocally:
        prefix = "CMD:TRCK:"
        ser.write(prefix + "QUAW:" + str(quat.w) + "\r\n")
        ser.write(prefix + "QUAX:" + str(quat.x) + "\r\n")
        ser.write(prefix + "QUAY:" + str(quat.y) + "\r\n")
        ser.write(prefix + "QUAZ:" + str(quat.z) + "\r\n")
        print "Requesting tracking (%s,%s,%s,%s)" % (quat.w, quat.x, quat.y, quat.z)

def serialLoop():
    global reqTorque
    global torqueSet
    global ser
    global reqNextStep
    global reqData
    
    s = str(ser.readline());
    
    if len(s) > 0:
        s = string.rstrip(s, "\r\n")
        print s
        if s[0:3]=="CMD":
            spl = s.split(':');
            print spl
            if spl[1]=="sendI":
                sendI()
            elif spl[1]=="TAUS":
                if spl[2]=="TAUX":
                    reqTorque[0] = float(spl[3])
                    torqueSet+=1;  
                elif spl[2]=="TAUY":
                    reqTorque[1] = float(spl[3])
                    torqueSet+=1;  
                elif spl[2]=="TAUZ":
                    reqTorque[2] = float(spl[3])
                    torqueSet+=1;  
            elif spl[1]=="power":
                #print "Power request for motor %s: %s" % (spl[2], spl[3])
                simu.b.changeInput(int(spl[2]), pow(float(spl[3]),2))
            elif spl[1]=="NEXT":
                reqNextStep += 1
            elif spl[1]=="REQD":
                reqData = True
            elif spl[1]=="REQT":
                sendTime()

def sendInstruction(cmd):
    prefix = "CMD:CTRL:"
    ser.write(prefix + cmd + "\r\n")
    
def main():
    global torqueSet
    global reqTorque
    global ser
    global reqNextStep
    global reqData
    timeout = 0.0001
    s = ""
    continuous = False
    tracking = False
    cont = False
    
    simu.initBody(Params.runLocally)
    if Params.runLocally==True:
        sendNewTracking([0, 0, 0])
    else:
        ser = serial.Serial(port=Params.comPort, baudrate=9600, timeout=1)        
    
    while(True):
        QtGui.QApplication.processEvents()
        s = readInput("", "", timeout)
         
        if Params.runLocally==False:
            serialLoop()
        if(continuous):
            if Params.runLocally==False:
                #if ser.inWaiting()==0 and reqNextStep==True:
                if reqNextStep>0:
                    #simu.setRequiredTorque(reqTorque)
                    torqueSet = 0
                    cont = True
                else:
                    cont = False
            else:
                cont = True
             
            if cont==True:
                if tracking:
                    sendNewTracking(None)
                simu.nextStep()
                reqNextStep -= 1
            if reqData:
                sendSensor()
                reqData=False
                 
        if s=="q":
            sys.exit(0)
        elif s=="c":
            continuous= not(continuous)
            reqTorque = Vector([0, 0 ,0])
            torqueSet = 3
            print "continuous %s" % (continuous)
        elif s=="s":
            simu.setRequiredTorque(reqTorque)
            if tracking:
                sendNewTracking(None)
            simu.nextStep()
            sendSensor()
        elif s=="d":
            simu.deviate()
            print "Disturbing system"
        elif s=="ms":
            simu.setMoveType(Params.moveType)
            tracking = True
        elif s=="ref":
            sendNewTracking([1,1,0])
        elif s=="calib":
            if Params.runLocally==True:
                simu.calibration()
            else:
                sendKValues()
                sendInstruction("GOCALIB")
        elif s=="dt":
            if simu.dt==0.1:
                simu.dt = 0.001
            else:
                simu.dt = 0.1
        elif s=="dump":
            simu.b.exportCalib()
        elif s=="load":
            simu.b.importCalib()
        elif s=="debug":
            sendDebug()
        elif s=="print":
            simu.test.plot()

if __name__ == "__main__":
    main()
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
    sys.exit(0);


