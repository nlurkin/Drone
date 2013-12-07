#!/usr/bin/env python
from DroneSimulator.matrix import *
import sys
import serial
import time, msvcrt
import qmath
import math



ser = serial.Serial(port=7, baudrate=9600, timeout=1)
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
        
    

if __name__ == "__main__":
    main()
    sys.exit(0);
