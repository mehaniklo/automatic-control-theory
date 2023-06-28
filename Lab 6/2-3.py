#!/usr/bin/env python3
from ev3dev.ev3 import LargeMotor
from ev3dev2.power import PowerSupply
import time
import os
import shutil
import math
import array

volts = PowerSupply()
motorA = LargeMotor('outA')

nameData = "data2-3"
# Создаем папочку для данных
if os.path.exists('/home/robot/'+nameData):
    shutil.rmtree('/home/robot/'+nameData)

os.makedirs('/home/robot/'+nameData)
A1 = 400
A2 = 100
w1 = 5
w2 = 2
phi1 = 6
phi2 = 3

def u1(uA, dt): 
    return uA[0]/dt
def u2(uA, dt): 
    return (2*uA[0] - uA[1])/(dt^2)
def u3(uA, dt): 
    return (3*uA[0] - 3*uA[1] + uA[2])/(dt^3)
def u4(uA, dt): 
    return (4*uA[0] - 6*uA[1] + 4*uA[2] - uA[3])/(dt^4)
def u5(uA, dt): 
    return (5*uA[0] - 10*uA[1] + 10*uA[2] - 5*uA[3] + uA[4])/(dt^4)

def e1(eA, dt): 
    return (eA[0] - eA[1])/dt
def e2(eA, dt): 
    return (eA[0] - 2*eA[1] + eA[2])/(dt^2)
def e3(eA, dt): 
    return (eA[0] - 3*eA[1] + 3*eA[2] - eA[3])/(dt^3)
def e4(eA, dt): 
    return (eA[0] - 4*eA[1] + 6*eA[2] - 4*eA[3] + eA[4])/(dt^4)
def e5(eA, dt): 
    return (eA[0] - 5*eA[1] + 10*eA[2] - 10*eA[3] + 5*eA[4] - eA[5])/(dt^4)

def addE(eA, cur):
    for i in range(5):
        eA[5 - i] = eA[5 - i - 1]
    eA[0] = cur
    return eA

def addU(uA, cur):
    for i in range(4):
        uA[4 - i] = uA[4 - i - 1]
    uA[0] = cur
    return uA

n1 = -14
n2 = -168
n3 = -239
n4 = -679
n5 = -593
n6 = 1

d1 = 1
d2 = 6
d3 = 29
d4 = 174
d5 = 100
d6 = 600
def getPer(cur, max):
    if (cur > max):
        return 100
    if (cur < -max):
        return -100
    return cur*100/max

uA = array.array('d', [0.0, 0.0, 0.0, 0.0, 0.0])
eA = array.array('d', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

try:
    start_time = time.time()
    start_pos = motorA.position
    while (time.time() - start_time) < 10:
        t = time.time()
        y = A1*math.cos(w1*(time.time() - start_time) + phi1) + A2*math.sin(w2*(time.time() - start_time) + phi2) + start_pos
        e = y - motorA.position
        eA = addE(eA, e)

        dt = time.time() - t 
        kT = (d1/dt^5 + d2/dt^4 + d3/dt^3 + d4/dt^2  + d5/dt  + d6)

        curU =  (d5*u1(eA, dt) + d4*u2(eA, dt) + d3*u3(eA, dt) + d2*u4(eA, dt) + d1*u5(eA, dt) + n6*e + n5*e1(eA, dt) + n4*e2(eA, dt) + n3*e3(eA, dt) + n2*e4(eA, dt) + n1*e5(eA, dt))/kT
        uA = addU(uA, curU)

        motorA.run_direct(duty_cycle_sp=getPer(curU, volts.measured_volts))
        with open(nameData+ '/' + nameData, "a") as f:
            print(str(time.time() - start_time) + ',' + str(motorA.position - start_pos) + ',' + str(motorA.speed))
            f.write(str(time.time() - start_time) + ',' + str(motorA.position - start_pos) + ',' + str(motorA.speed) + '\n')
    motorA.run_direct(duty_cycle_sp=0)
    time.sleep(1)
except Exception as e:
    raise e
finally:
    motorA.stop(stop_action='brake')
print("Volts: " + str(volts.measured_volts))