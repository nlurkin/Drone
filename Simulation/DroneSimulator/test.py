#!/usr/bin/env python
import matplotlib.pyplot as plt
from main import readInput
from FullSimu import Simu

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