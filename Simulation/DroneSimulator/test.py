#!/usr/bin/env python
from mathclasses import Quaternion
from scipy.constants.constants import pi


q = Quaternion([pi/2, pi, 0])

print q
print q.real()
print q.imag()
print q.conj()

'''
simu = Simu()

t = 0
dt = 0.1

simu.plotSetup()
simu.setMoveType(3)

s = ""
while s!="p":
    s = readInput("", "", 0.0001)
    plt.pause(1)

while t<10:
    simu.getNextMove()
    plt.pause(0.0001)
'''