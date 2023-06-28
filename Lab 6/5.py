#!/usr/bin/env python3
from ev3dev.ev3 import LargeMotor
from ev3dev2.power import PowerSupply
import time
import os
import shutil
import math

volts = PowerSupply()
motorA = LargeMotor('outA')

nameData = "data3"
# РЎРѕР·РґР°РµРј РїР°РїРѕС‡РєСѓ РґР»СЏ РґР°РЅРЅС‹С…
if os.path.exists('/home/robot/'+nameData):
    shutil.rmtree('/home/robot/'+nameData)

os.makedirs('/home/robot/'+nameData)

A1 = 500
w1 = 2
A2 = 70
A3 = 300
w2 = 4
w3 = 7
def getPer(cur, max):
    if (cur > max):
        return 100
    if (cur < -max):
        return -100
    return cur*100/max

try:
    nameFile = " " + str(A1) + " " + str(w1)
    start_time = time.time()
    start_pos = motorA.position
    while (time.time() - start_time) < 10:
        U = A1*math.sin(w1*(time.time() - start_time))
        motorA.run_direct(duty_cycle_sp=getPer(U, 7))
        with open(nameData+ '/' + nameData + nameFile, "a") as f:
            print(str(time.time() - start_time) + ',' + str(motorA.position - start_pos) + ',' + str(motorA.speed))
            f.write(str(time.time() - start_time) + ',' + str(motorA.position - start_pos) + ',' + str(motorA.speed) + '\n')
    motorA.run_direct(duty_cycle_sp=0)
    time.sleep(1)

    nameFile = " " + str(A2) + " " + str(w2) + " " + str(A3) + " " + str(w3)
    start_time = time.time()
    start_pos = motorA.position
    while (time.time() - start_time) < 10:
        U = A2*math.cos(w2*(time.time() - start_time)) + A3*math.sin(w3*(time.time() - start_time))
        motorA.run_direct(duty_cycle_sp=getPer(U, volts.measured_volts))
        with open(nameData+ '/' + nameData + nameFile, "a") as f:
            print(str(time.time() - start_time) + ',' + str(motorA.position - start_pos) + ',' + str(motorA.speed))
            f.write(str(time.time() - start_time) + ',' + str(motorA.position - start_pos) + ',' + str(motorA.speed) + '\n')
    motorA.run_direct(duty_cycle_sp=0)
    time.sleep(1)
except Exception as e:
    raise e
finally:
    motorA.stop(stop_action='brake')
print("Volts: " + str(volts.measured_volts))