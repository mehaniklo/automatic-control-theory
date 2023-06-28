#!/usr/bin/env python3
from ev3dev.ev3 import LargeMotor
from ev3dev2.power import PowerSupply
import time
import os
import shutil

volts = PowerSupply()
motorA = LargeMotor('outA')
def getPer(cur, max):
    if (cur > max):
        return 100
    if (cur < -max):
        return -100
    return cur*100/max

nameData = "2pi"
# Создаем папочку для данных
if os.path.exists('/home/robot/'+nameData):
    shutil.rmtree('/home/robot/'+nameData)

os.makedirs('/home/robot/'+nameData)
kp = 1
ki = 1

try:
    start_time = time.time()
    start_pos = motorA.position
    I = 0
    while (time.time() - start_time) < 2:
        t = time.time()
        y = (time.time() - start_time) + start_pos
        e = y - motorA.position
        dt = time.time() - t 
        I += dt * e
        U = e*kp + I*ki
        motorA.run_direct(duty_cycle_sp=getPer(U, volts.measured_volts))
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