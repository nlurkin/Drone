#!/usr/bin/env python
from FullSimu import Simu
from matrix import *
import math
import msvcrt
import qmath
import serial
import string
import sys
from time import sleep, time

acc = [0, 0, 0]
gyro = [0, 0, 0]
angle = [0, 0, 0]
quat = qmath.quaternion


ser = serial.Serial(port=7, baudrate=9600, timeout=1)

simu = Simu()

def readInput( caption, default, timeout = 5):
    start_time = time()
    #sys.stdout.write('%s(%s):'%(caption, default));
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

    #print ''  # needed to move to next line
    if len(inputS) > 0:
        return inputS
    else:
        return default

def buildAttitude(alpha, beta, gamma, time):
    global gyro
    global angle
    global quat
    
    gyro = [(alpha-angle[0])/time, (beta-angle[1])/time, (gamma-angle[2])/time]
    angle = [alpha, beta, gamma]
    quat = qmath.quaternion([alpha, beta, gamma])

def sendSensor():
    prefix = "DAT:SENS:"
    time = 0.05
    ser.write(prefix + "BUF0:" + str(quat[0]) + "\r\n")
    sleep(time);
    ser.write(prefix + "BUF1:" + str(quat[1]) + "\r\n")
    sleep(time);
    ser.write(prefix + "BUF2:" + str(quat[2]) + "\r\n")
    sleep(time);
    ser.write(prefix + "BUF3:" + str(quat[3]) + "\r\n")
    sleep(time);
    ser.write(prefix + "BUF4:" + str(int(gyro[0]*131*2)) + "\r\n")
    sleep(time);
    ser.write(prefix + "BUF5:" + str(int(gyro[1]*131*2)) + "\r\n")
    sleep(time);
    ser.write(prefix + "BUF6:" + str(int(gyro[2]*131*2)) + "\r\n")
    sleep(time);
    ser.write(prefix + "BUF7:" + str(int(acc[0]*8192*2)) + "\r\n")
    sleep(time);
    ser.write(prefix + "BUF8:" + str(int(acc[1]*8192*2)) + "\r\n")
    sleep(time);
    ser.write(prefix + "BUF9:" + str(int(acc[2]*8192*2)) + "\r\n")
    print "Sending data"
    
def sendI():
    I = simu.getI()
    prefix = "DAT:IMAT:"
    ser.write(prefix + "IXX:" + str(I[0]) + "\r\n")
    ser.write(prefix + "IYY:" + str(I[1]) + "\r\n")
    ser.write(prefix + "IZZ:" + str(I[2]) + "\r\n")
    print "Sending %s %s %s" % (I[0], I[1], I[2])

def loop():
    s = str(ser.readline());
    
    if len(s) > 0:
        print s
        if s[0:3]=="CMD":
            spl = s.split(':');
            print spl
            if string.rstrip(spl[1], "\r\n")=="sendI":
                sendI()
            #if s[1]=="power":
                #changeMotorPower(s[2], s[3])


def main():
    timeout = 0.0001
    s = ""
    while(True):
        loop()
        s = readInput("", "", timeout)
        print s
        if s=="q":
            break
        elif s=="s":
            buildAttitude(0, math.pi/4, 0, 1)
            sendSensor()
        
    


if __name__ == "__main__":
    main()
    sys.exit(0);





'''
acc = [0, 0, 0]
gyro = [0, 0, 0]
angle = [0, 0, 0]
quat = qmath.quaternion

I = [[1, 2, 3], [4, 5, 6], [7, 8, 9]]
R = [[1, 1, 1], [2, 2, 2], [3, 3, 3], [4, 4, 4]]


def SR(alpha, R, component):
    return alpha[0]*R[0][component] + alpha[1]*R[1][component]  + alpha[1]*R[1][component]

def SI(alpha, component):
    global I
    return alpha[0]*I[component][0] + alpha[1]*I[component][1] + alpha[1]*I[component][1] 
    
def buildOmega(moteur, power, alpha):
    omega = [0, 0, 0]
    
    omega[0] = () - 

def buildTorque(moteur, power, alpha):
    global I
    omega = buildOmega(moteur, power, alpha)
    torque = vecSum(matDotProduct(I, alpha), vecCrossProduct(omega, matDotProduct(I, omega)))
    return torque
        
def readInput( caption, default, timeout = 5):
    start_time = time.time()
    #sys.stdout.write('%s(%s):'%(caption, default));
    inputS = ''
    while True:
        if msvcrt.kbhit():
            chrS = msvcrt.getche()
            if ord(chrS) == 13: # enter_key
                break
            elif ord(chrS) >= 32: #space_char
                inputS += chrS
        if len(inputS) == 0 and (time.time() - start_time) > timeout:
            break

    #print ''  # needed to move to next line
    if len(inputS) > 0:
        return inputS
    else:
        return default

def buildAttitude(alpha, beta, gamma, time):
    global gyro
    global angle
    global quat
    
    gyro = [(alpha-angle[0])/time, (beta-angle[1])/time, (gamma-angle[2])/time]
    angle = [alpha, beta, gamma]
    quat = qmath.quaternion([alpha, beta, gamma])
    
def sendSensor():
    ser.write(str(quat[0]) + "\r\n")
    ser.write(str(quat[1]) + "\r\n")
    ser.write(str(quat[2]) + "\r\n")
    ser.write(str(quat[3]) + "\r\n")
    ser.write(str(int(gyro[0]*131*2)) + "\r\n")
    ser.write(str(int(gyro[1]*131*2)) + "\r\n")
    ser.write(str(int(gyro[2]*131*2)) + "\r\n")
    ser.write(str(int(acc[0]*8192*2)) + "\r\n")
    ser.write(str(int(acc[1]*8192*2)) + "\r\n")
    ser.write(str(int(acc[2]*8192*2)) + "\r\n")
    
def changeMotorPower(motor, power):
    buildAttitude(alpha, beta, gamma, time)
    
    
def loop():
    s = str(ser.readline());
    
    if len(s) > 0:
        print s
        if s[0:3]=="CMD":
            s.split(':');
            if s[1]=="power":
                changeMotorPower(s[2], s[3])

def main():
    timeout = 0.0001
    s = ""
    while(True):
        loop()
        s = readInput("", "", timeout)
        if s=="q":
            break
        elif s=="s":
            buildAttitude(0, math.pi/4, 0, 1)
            sendSensor()
        
    

'''