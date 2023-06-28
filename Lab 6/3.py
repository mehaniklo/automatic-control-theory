#!/usr/bin/env python3
from ev3dev.ev3 import LargeMotor
from ev3dev2.power import PowerSupply
import time
import os
import shutil
import math
def getPer(cur, max):
    if (cur > max):
        return 100
    if (cur < -max):
        return -100
    return cur*100/max

volts = PowerSupply()
motorA = LargeMotor('outA')

nameData = "data3"
# Создаем папочку для данных
if os.path.exists('/home/robot/'+nameData):
    shutil.rmtree('/home/robot/'+nameData)

os.makedirs('/home/robot/'+nameData)

try:
    for i in range(19):
        start_time = time.time()
        t = time.time()
        start_pos = motorA.position
        while (time.time() - start_time) < 10:
            U = 800*math.sin((i+1)*(time.time() - start_time))
            motorA.run_direct(duty_cycle_sp=getPer(U, volts.measured_volts))
            with open(nameData+ '/' + nameData + str(i+1), "a") as f:
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