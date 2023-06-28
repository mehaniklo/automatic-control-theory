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

# Создаем папочку для данных
if os.path.exists('/home/robot/data_2p'):
    shutil.rmtree('/home/robot/data_2p')

motorA.position = 0

os.makedirs('/home/robot/data_2p')
kp = 1.4
try:
    for i in range(3):
        start_time = time.time()
        start_pos = motorA.position
        # В течение двух секунд записываем в файл данные в формате: <time,angle,speed>
        while (time.time() - start_time) < 10:
            y = 9
            e = y - motorA.position
            u = e*kp*(i+1)
            motorA.run_direct(duty_cycle_sp=getPer(u, volts.measured_volts))
            with open('data_2p/2p_data'+str(i+1), "a") as f:
                print(str(time.time() - start_time) + ',' + str(motorA.position - start_pos) + ',' + str(motorA.speed))
                print(str(time.time() - start_time) + ',' + str(e))
                f.write(str(time.time() - start_time) + ',' + str(motorA.position - start_pos) + ',' + str(motorA.speed) + '\n')
        motorA.run_direct(duty_cycle_sp=0)
        time.sleep(1)
except Exception as e:
    raise e
finally:
    motorA.stop(stop_action='brake')
print("Volts: " + str(volts.measured_volts))