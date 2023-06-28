#!/usr/bin/env python3
from ev3dev.ev3 import LargeMotor
from ev3dev2.power import PowerSupply
import time
import os
import shutil
import array

volts = PowerSupply()
motorA = LargeMotor('outA')

nameData = "data4-0.6"
# РЎРѕР·РґР°РµРј РїР°РїРѕС‡РєСѓ РґР»СЏ РґР°РЅРЅС‹С…
if os.path.exists('/home/robot/'+nameData):
    shutil.rmtree('/home/robot/'+nameData)

os.makedirs('/home/robot/'+nameData)


def UpperBound(A, key, left, right): 
    while right > left + 1: 
        middle = (left + right) // 2 
        if A[middle] > key: 
            right = middle 
        else: 
            left = middle 
    return right
kp = 0.05
errors = array.array('d',[0.0])
times = array.array('d',[0.0])
left = -1 
T = 0.6
def getPer(cur, max):
    if (cur > max):
        return 100
    if (cur < -max):
        return -100
    return cur*100/max

try:
    start_time = time.time()
    t = time.time()
    start_pos = motorA.position
    y = 1000 + start_pos
    while (time.time() - start_time) < 20:
        e = y - motorA.position
        errors.append(e)
        times.append(time.time() - start_time)

        timeOld = time.time() - start_time - T
        oldIndex = 1
        right = 0
        if (timeOld > 0):
            oldIndex = UpperBound(times, timeOld, left, len(times))
            left = oldIndex
        oldE = errors[oldIndex-1]
        U = oldE*kp
        motorA.run_direct(duty_cycle_sp=getPer(U, volts.measured_volts))
        with open(nameData+ '/' + nameData, "a") as f:
            print(str(time.time() - start_time) + ',' + str(motorA.position - start_pos) + ',' + str(motorA.speed))
            f.write(str(time.time() - start_time) + ',' + str(motorA.position - start_pos) + ',' + str(motorA.speed) + '\n')
        t = time.time()
    motorA.run_direct(duty_cycle_sp=0)
    time.sleep(1)
except Exception as e:
    raise e
finally:
    motorA.stop(stop_action='brake')
print("Volts: " + str(volts.measured_volts))

