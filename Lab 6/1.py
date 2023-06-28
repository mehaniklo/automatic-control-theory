#!/usr/bin/env python3
from ev3dev.ev3 import LargeMotor
from ev3dev2.power import PowerSupply
import time
import os
import shutil

# Класс для получения данных о заряде батареи
volts = PowerSupply()
# Класс для взаимодействия с двигателем
motorA = LargeMotor('outA')

# Создаем папочку для данных
if os.path.exists('/home/robot/data_mnk'):
    shutil.rmtree('/home/robot/data_mnk')

os.makedirs('/home/robot/data_mnk')

try:
    for i in range(5):
        # Задаем входное напряжение, которое будет меняться от 20% до 100% с шагом 20
        U = (i + 1)*20
        # Сохраняем начальное время
        start_time = time.time()
        # Сохраняем начальную позицию
        start_pos = motorA.position
        # Подаем на мотор напряжение U
        motorA.run_direct(duty_cycle_sp=U)
        # В течение двух секунд записываем в файл данные в формате: <time,angle,speed>
        while (time.time() - start_time) < 2:
            # Конструкция 'with open() as f' означает то же самое, что и 'f = open(); ...; f.close()'
            with open('data_mnk/mnk_data'+str(U), "a") as f:
                print(str(time.time() - start_time) + ',' + str(motorA.position - start_pos) + ',' + str(motorA.speed))
                f.write(str(time.time() - start_time) + ',' + str(motorA.position - start_pos) + ',' + str(motorA.speed) + '\n')
        # Останавливаем мотор
        motorA.run_direct(duty_cycle_sp=0)
        time.sleep(1)
        # Повторяем то же самое для отрицательного напряжения
        start_time = time.time()
        start_pos = motorA.position
        motorA.run_direct(duty_cycle_sp=-U)
        while (time.time() - start_time) < 2:
            with open('data_mnk/mnk_data'+str(-U), "a") as f:
                print(str(time.time() - start_time) + ',' + str(motorA.position - start_pos) + ',' + str(motorA.speed))
                f.write(str(time.time() - start_time) + ',' + str(motorA.position - start_pos) + ',' + str(motorA.speed) + '\n')
        motorA.run_direct(duty_cycle_sp=0)
        time.sleep(1)
except Exception as e:
    raise e
finally:
    # Останавливаем мотор в случае ошибок в коде
    motorA.stop(stop_action='brake')

# Выводим заряд батареи, то есть реальное напряжение в вольтах, которое подается на двигатель при 100% поданной на него мощности
print("Volts: " + str(volts.measured_volts))