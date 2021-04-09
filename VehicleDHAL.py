#!/usr/bin/env python3
import sys

from b0lib import b0RemoteApi
import time
import math
from messages import Pose
from messages import Vector3

__email__ = "taur.os.iv@gmail.com"

"""
#################################################################
#      Класс обертка для работы с моделью робота в CoppeliaSim.
#      Режим b0Api
#################################################################


Офф. документация по встроенным функциям (Regular API LUA): https://www.coppeliarobotics.com/helpFiles/index.html
Офф. документация по B0-based remote API:
- Настройка на стороне сервера https://www.coppeliarobotics.com/helpFiles/en/b0RemoteApiServerSide.htm
- Настройка на стороне клиента https://www.coppeliarobotics.com/helpFiles/en/b0RemoteApiClientSide.htm
- Справочник функций  https://www.coppeliarobotics.com/helpFiles/en/b0RemoteApi-functionList.htm
- Минимальный пример https://www.coppeliarobotics.com/helpFiles/en/b0RemoteApiModusOperandi.htm


Настройки в CoppeliaSim.
- На сцене для Joint включить Tourque|Force -> MotorEnafled -> 0,0 
- Для опознавания VisualSensor у объекта "Scene Object Properies -> Common -> Object Special Properties -> Renderable"
- Для опознования ProximitySensor у объекта "Scene Object Properies -> Common -> Object Special Properties -> Detectable"
- Для расчета столкновений, опор у объекта пол/плитка "Scene Object Properies -> Shape -> Show dynamic properties dialog"
  вкл. Body is respondable
  откл. Body is dynamic

Для работы необходимы файлы библиотеки (py и *.dll(Windows) / *.so(Linux)),
Добавить следующие в папку с исполняемым *.py:
    b0.py           из /opt/V-REP_PRO_EDU/programming/b0RemoteApiBindings/python/b0.py
    b0RemoteApi.py  из /opt/V-REP_PRO_EDU/programming/b0RemoteApiBindings/python/b0RemoteApi.py
    b0.dll/.so      из /opt/V-REP_PRO_EDU/libb0.so
    boost_date_time-vc141-mt-x64-1_70.dll/.so
    boost_filesystem-vc141-mt-x64-1_70.dll/.so
    boost_program_options-vc141-mt-x64-1_70.dll/.so
    boost_regex-vc141-mt-x64-1_70.dll/.so
    boost_thread-vc141-mt-x64-1_70.dll/.so
    libzmq-mt-4_3_2.dll/.so
    lz4.dll/.so
    zlib1.dll/.so

Доп. параметр при чтении или записи: 
    - Функции типа client.simxGet...(..., self.client.simxServiceCall())
    - Функции типа client.simxSet...(..., self.client.simxDefaultPublisher())
    - Функции типа client.simxSet...(..., self.client.simxDefaultSubscriber(self.CALLBACK_FUNCTION))

TODO:
    + Энкодеры моторов
    + Акселерометр x,y,z
    + Гироскоп x,y,z
    + Вычисление скорости
    +- Точное положение 
    + Перевести с радиан +Pi -PI в градусы 0 ... 360 
"""


class VehicleDHAL:
    """
    Класс обертка для работы с моделью двухколёсного робота в  CoppeliaSim.
    """

    # -------------------------------------------------------------------------------------------------------------------------------
    def __init__(self,
                 b0ChannelName,
                 runInSynchMode=True,
                 maxSpeedWheel_meterPerSec=0.2,
                 isProxSensorActive=True,
                 isVisSensorActive=True,
                 isAccelActive=True,
                 isGyroActive=True,
                 isVerbose=False
                 ):
        """
        Конструктор
        :param b0ChannelName: Название "канала" для связи с b0-CoppeliaSim
        :param maxSpeedWheel_degrePerSec: Макс. желаемая скорость м/с
        :param runInSynchMode:    Флаг работы симулятора в синхронном режиме
        :param isVisSensorActive: Флаг активации обработчиков визуальных сенсенов
        :param isAccelActive:     Флаг активации обработчиков акселерометра
        :param isGyroActive:      Флаг активации обработчиков гироскопа
        :param isVerbose:         Флаг вывода некоторых отладочных сообщений
        """
        #@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        self.calcErrorLeftABS_pos_Deg_Total  = 0
        self.calcErrorRightABS_pos_Deg_Total = 0
        #@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@


        print('__init__')
        self.isVerbose = isVerbose
        self.b0ChannelName = b0ChannelName

        # ------------------------------------------------------------------------------------------------------------------
        # Настройка клиента b0RemoteApi
        self.client = b0RemoteApi.RemoteApiClient('b0RemoteApi_pythonClient', self.b0ChannelName)
        self.client.doNextStep = True  # флаг разрешения выполнения шага симуляции
        self.client.runInSynchMode = True  # флаг работы в синхронном режиме

        if self.client != -1:
            # При успешном соединении, вывод сообщения в симуляторе
            print("+++ Соединение УСТАНОВЛЕНО с удаленным b0-API-server +++")
            self.client.simxAddStatusbarMessage(
                "<br><br> <font color=\'red\'>+++ Соединение с управляющай Python-программой: УСТАНОВЛЕНО  +++</font>@html",
                self.client.simxDefaultPublisher())
        else:
            print("!!! Соединение с удаленным b0-API-server НЕУДАЛОСЬ !!!")
            sys.exit()

        # -------------------------------------------------------------------------------------------------------------
        self.simTime = 0  # Тек. метка времени работы симуляции, сек
        self.simTimePrev = 0  # Предыдущая метка времени работы симуляции, сек
        self.realStartTime = 0  # Начальная метка времени Реальная, сек

        # Физические параметры модели.---------------------------------------------------------------------------------
        self.wheelDiametr = 0.07  # Диаметр колес
        self.L = 0.16  # ширина колесной базы (между центрами)
        self.maxForce = 10.5  # Крутящий момент моторов 0.5 [N *m] ~= 5 [N *cm]
        self.basePosition = Pose()  # положение и ориентация модели в пространстве симулятора
        self.basePositionPrev = Pose()  # положение и ориентация модели в пространстве симулятора
        self.accel = Vector3(0, 0, 0)  # показания акселерометра (линейное ускорение, м/сек^2)
        self.gyro = Vector3(0, 0, 0)  # показания гироскопа (угловая скорость, град/сек ? рад/сек)

        # Расчет скорости ---------------------------------------------------------------------------------------------
        self.wL = 0  # циклическая скорость правого колеса
        self.wR = 0  # циклическая скорость левого колеса
        self.vL = 0  # линейная скорость колеса
        self.vR = 0  # линейная скорость колеса
        self.vRobot = 0  # линейная скорость модели
        self.wRobot = 0  # циклическая скорость модели (поворот вокруг центральной оси)

        # Расчет скорости вращения колес в гр/сек для достижения целевой м/с (maxSpeedWheel_meterPerSec)
        # Пример исп: simxSetJointTargetVelocity( 2pi  [rad/sec]) -> На стороне Coppelia (360 [deg/sec])
        # Макс Скорость вращение колес   [deg/sec]
        self.maxSpeedWheel_degrePerSec = (maxSpeedWheel_meterPerSec * 360) / (math.pi * self.wheelDiametr)

        # Макс Скорость вращение колес   [rad/sec] исп. simxSetJointTargetVelocity
        self.maxSpeedWheel_Rad = self.maxSpeedWheel_degrePerSec / 180.0 * math.pi

        # Скорость вращения датчика расстояния
        ##self.maxSpeedSensor_degrePerSec = 10
        ##self.maxSpeedSensor_radPerSec   = (math.pi/180.0) * self.maxSpeedSensor_degrePerSec

        print(
            "\nСкорость макс.:\n  - вращения колес макс.      {:7.2f} град/сек, {:7.2} рад/сек \n  - передвижения модели макс. {:7.2f} м/сек\n".format(
                self.maxSpeedWheel_degrePerSec, self.maxSpeedWheel_Rad,
                maxSpeedWheel_meterPerSec))

        self.wheelSpeed_L_Rad = 0  # целевая скорость цилцическая левого  колеса [rad/sec]
        self.wheelSpeed_R_Rad = 0  # целевая скорость цилцическая правого колеса [rad/sec]
        self.wheelSpeedReal_L_Rad = 0  # факт вычисленная
        self.wheelSpeedReal_R_Rad = 0  # факт вычисленная

        # Показания датчиков -----------------------------------------------------------------------------------------
        self.sensVisL = 0  # среднее значение яркости кадра визуального датчика. avg intensivity white from Visual sensor
        self.sensVisR = 0  # среднее значение яркости кадра визуального датчика. avg intensivity white from Visual sensor

        # Положение мотора - энкодер (Градусы / Радианы)
        self.revJ_wheelL_pos_deg = 0  # Тек. положение. Градусы (диапазон 0 +180 ... -180 ... 0)
        self.revJ_wheelR_pos_deg = 0  # Тек. положение. Градусы (диапазон 0 +180 ... -180 ... 0)
        self.revJ_wheelL_pos_rad = 0  # Тек. положение. Радианы (диапазон 0 +PI ... -PI ... 0)
        self.revJ_wheelR_pos_rad = 0  # Тек. положение. Радианы (диапазон 0 +PI ... -PI ... 0)
        self.revJ_wheelL_pos_deg_Prev = 0  # Положение на пред. шаге симуляции
        self.revJ_wheelR_pos_deg_Prev = 0  # Положение на пред. шаге симуляции
        self.revJ_wheelL_pos_rad_Prev = 0  # Положение на пред. шаге симуляции
        self.revJ_wheelR_pos_rad_Prev = 0  # Положение на пред. шаге симуляции
        self.TTT = 0
        # Положение мотора - энкодер. Накопленное значение угла поворота
        self.revJ_wheelL_pos_deg_Total = 0  # Накопленное. Градусы (диапазон 0 +180 ... -180 ... 0)
        self.revJ_wheelR_pos_deg_Total = 0  # Накопленное. Градусы (диапазон 0 +180 ... -180 ... 0)
        self.revJ_wheelL_pos_rad_Total = 0  # Накопленное. Радианы (диапазон 0 +PI ... -PI ... 0)
        self.revJ_wheelR_pos_rad_Total = 0  # Накопленное. Радианы (диапазон 0 +PI ... -PI ... 0)

        self.revJ_wheelL_pos_deg_Delta =0 #Прирост к повороту колеса/энкодеру
        self.revJ_wheelR_pos_deg_Delta =0 #Прирост к повороту колеса/энкодеру

        # Положение-направление датчика расстояния. Абсолютное значение положение
        self.revJ_sensor_pos_deg_ABS = 0  # Градусы
        self.revJ_sensor_pos_rad_ABS = 0  # Радианы

        # Расстояние до препятствия. Датчик расстояния ультразвук
        self.sensDistance_F = 0 #qqq
        self.sensDistance_S = 0

        # ПИД регулятор ходовых моторов ------------------------------------------------------------------------------
        self.kProp = 1.0 * 100.0
        self.kDiff = 0.4 * 100.0
        self.prevErr = 0
        self.nowErr = 0

        # -------------------------------------------------------------------------------------------------------------
        # -------------------------------------------------------------------------------------------------------------
        # Привязка функций обработчиков

        # Обработчики кадра симуляции
        self.client.simxGetSimulationStepStarted(self.client.simxDefaultSubscriber(self.simulationStepStarted))
        self.client.simxGetSimulationStepDone(self.client.simxDefaultSubscriber(self.simulationStepDone))

        # --------------------------------------------------------------------------------------------------------------
        # Блок привязки к узлам конкретной модели в CoppeliaSim
        dPartsName = {
            "base": "vehicleBase",  # Базовый родительский объект модели
            "revJ_WL": "revJoint_wheelL",  # Поворотное соединение Колесо левое
            "revJ_WR": "revJoint_wheelR",  # Поворотное соединение Колесо правое
            "sensVisL": "Vision_sensorL",  # Датчик освещенности (на основе датчика зрения)
            "sensVisR": "Vision_sensorR",  # Датчик освещенности (на основе датчика зрения)

            "sensProx1":  "Proximity_sensor_F",  # Датчик расстояния
            "revJ_Prox1": "revJoint_sensor_F",   # Поворотное соединение датчика расстояния

            "sensProx2":  "Proximity_sensor_S",  # Датчик расстояния
            "revJ_Prox2": "revJoint_sensor_S",  # Поворотное соединение датчика расстояния
        }

        # Получить Хэндлеры-ID узов со стороны Coppelia
        # revJ = revoluteJoint = Вращающееся соединение
        # _H = Handler
        # proxSen = поворотное датчика
        # wheelL = Ходовые моторы
        strErrPartNotFound = "!!! Не найден узел модели с названием "
        isNoErr_base, self.base = self.client.simxGetObjectHandle(dPartsName["base"], self.client.simxServiceCall())
        isNoErr_rjwl, self.revJ_wheelL_H = self.client.simxGetObjectHandle(dPartsName["revJ_WL"], self.client.simxServiceCall())
        isNoErr_rjwr, self.revJ_wheelR_H = self.client.simxGetObjectHandle(dPartsName["revJ_WR"], self.client.simxServiceCall())
        isNoErr_vsl, self.visSensorL_H = self.client.simxGetObjectHandle(dPartsName["sensVisL"], self.client.simxServiceCall())
        isNoErr_vsr, self.visSensorR_H = self.client.simxGetObjectHandle(dPartsName["sensVisR"], self.client.simxServiceCall())

        isNoErr_prs1, self.proxSens_H_1 = self.client.simxGetObjectHandle(dPartsName["sensProx1"], self.client.simxServiceCall())
        isNoErr_rjs1, self.revJ_sensor_H_1 = self.client.simxGetObjectHandle(dPartsName["revJ_Prox1"], self.client.simxServiceCall())

        isNoErr_prs2, self.proxSens_H_2 = self.client.simxGetObjectHandle(dPartsName["sensProx2"], self.client.simxServiceCall())
        isNoErr_rjs2, self.revJ_sensor_H_2 = self.client.simxGetObjectHandle(dPartsName["revJ_Prox1"], self.client.simxServiceCall())

        # проверка наличия всех узлов модели
        assert isNoErr_base, (strErrPartNotFound + dPartsName["base"])
        assert isNoErr_rjwl, (strErrPartNotFound + dPartsName["revJ_WL"])
        assert isNoErr_rjwr, (strErrPartNotFound + dPartsName["revJ_WR"])
        assert isNoErr_vsl, (strErrPartNotFound + dPartsName["sensVisL"])
        assert isNoErr_vsr, (strErrPartNotFound + dPartsName["sensVisR"])

        assert isNoErr_prs1, (strErrPartNotFound + dPartsName["sensProx1"])
        assert isNoErr_rjs1, (strErrPartNotFound + dPartsName["revJ_Prox1"])
        assert isNoErr_prs2, (strErrPartNotFound + dPartsName["sensProx2"])
        assert isNoErr_rjs2, (strErrPartNotFound + dPartsName["revJ_Prox2"])

        # назначить обработчики Callback для поступающих значений
        self.client.simxGetObjectPosition(self.base, -1, self.client.simxDefaultSubscriber(self.basePositionCallBack))  # -1 для запроса абсолютных координат
        self.client.simxGetObjectOrientation(self.base, -1, self.client.simxDefaultSubscriber(self.baseOrientationCallBack))  # -1 для запроса абсолютных координат
        self.client.simxGetJointPosition(self.revJ_wheelL_H, self.client.simxDefaultSubscriber(self.revJ_wheelL_ReadAngle_Callback))
        self.client.simxGetJointPosition(self.revJ_wheelR_H, self.client.simxDefaultSubscriber(self.revJ_wheelR_ReadAngle_Callback))

        if isProxSensorActive:
            self.client.simxReadProximitySensor(self.proxSens_H_1, self.client.simxDefaultSubscriber(self.ultrasonicReadCallBack))
            self.client.simxGetJointPosition(self.revJ_sensor_H_1, self.client.simxDefaultSubscriber(self.revJ_sensor_ReadAngle_Callback))

            self.client.simxReadProximitySensor(self.proxSens_H_2, self.client.simxDefaultSubscriber(self.ultrasonicReadCallBack_2))
            ###self.client.simxGetJointPosition(self.revJ_sensor_H_2, self.client.simxDefaultSubscriber(self.revJ_sensor_ReadAngle_Callback))

            self.client.simxSetJointMaxForce(self.revJ_sensor_H_1, self.maxForce, self.client.simxDefaultPublisher())
            self.client.simxSetJointTargetVelocity(self.revJ_sensor_H_1, 0, self.client.simxDefaultPublisher())

            self.client.simxSetJointMaxForce(self.revJ_sensor_H_2, self.maxForce, self.client.simxDefaultPublisher())
            self.client.simxSetJointTargetVelocity(self.revJ_sensor_H_2, 0, self.client.simxDefaultPublisher())


        if isVisSensorActive:
            self.client.simxReadVisionSensor(self.visSensorL_H, self.client.simxDefaultSubscriber(self.visionReadCallbackL))
            self.client.simxReadVisionSensor(self.visSensorR_H, self.client.simxDefaultSubscriber(self.visionReadCallbackR))

        if isGyroActive:
            self.client.simxGetFloatSignal("gyroX", self.client.simxDefaultSubscriber(self.geGyroX_Callback))  # Гироскоп X
            self.client.simxGetFloatSignal("gyroY", self.client.simxDefaultSubscriber(self.geGyroY_Callback))  # Гироскоп Y
            self.client.simxGetFloatSignal("gyroZ", self.client.simxDefaultSubscriber(self.geGyroZ_Callback))  # Гироскоп Z
        if isAccelActive:
            self.client.simxGetFloatSignal("accelX", self.client.simxDefaultSubscriber(self.geAccelX_Callback))  # Акселерометр X
            self.client.simxGetFloatSignal("accelY", self.client.simxDefaultSubscriber(self.geAccelY_Callback))  # Акселерометр Y
            self.client.simxGetFloatSignal("accelZ", self.client.simxDefaultSubscriber(self.geAccelZ_Callback))  # Акселерометр Z
        #####self.client.simxGetStringSignal("image", self.client.simxDefaultSubscriber(self.image_Callback))

        # Установить параметры моторов
        # Макс.  Крутящий момент моторов
        self.client.simxSetJointMaxForce(self.revJ_wheelL_H, self.maxForce, self.client.simxDefaultPublisher())
        self.client.simxSetJointMaxForce(self.revJ_wheelR_H, self.maxForce, self.client.simxDefaultPublisher())

        # Текущая скорость вращения соединений
        self.client.simxSetJointTargetVelocity(self.revJ_wheelL_H, 0, self.client.simxDefaultPublisher())
        self.client.simxSetJointTargetVelocity(self.revJ_wheelR_H, 0, self.client.simxDefaultPublisher())

    pass

    # ------------------------------------------------------------------------
    def __del__(self):
        if self.isVerbose: print('Destructor called, Employee deleted.')
        # self.client._node.cleanup()
        # self.client.__exit__()
        # del self.client

    def __delete__(self, instance):
        if self.isVerbose: print("Inside __delete__")

    # -------------------------------------------------------------------------------------------------------------------------------

    #########################################################################################
    #  Обработчики Callback
    #########################################################################################

    def baseOrientationCallBack(self, msg):
        """ Получение углов поворота В ГРАДУСАХ """
        #   изменяет значения радианы 0  ... +3.13 +3.14 | -3.14 -3.13 ...  0
        #   изменяет значения градусы 0 +1 ... +179 +180 | -180 -179 ... -1 0
        if (msg[0] == True and len(msg[1]) == 3):
            self.basePosition.orientation.x = (msg[1][0] * 180) / math.pi
            self.basePosition.orientation.y = (msg[1][1] * 180) / math.pi
            self.basePosition.orientation.z = (msg[1][2] * 180) / math.pi

    def basePositionCallBack(self, msg):
        """ Получение абсолютных координат        """
        if (msg[0] == True and len(msg[1]) == 3):
            self.basePositionPrev.position.x = self.basePosition.position.x
            self.basePositionPrev.position.y = self.basePosition.position.y
            self.basePositionPrev.position.z = self.basePosition.position.z
            self.basePositionPrev.orientation.x = self.basePosition.orientation.x
            self.basePositionPrev.orientation.y = self.basePosition.orientation.y
            self.basePositionPrev.orientation.z = self.basePosition.orientation.z
            self.basePositionPrev.orientation.w = self.basePosition.orientation.w

            self.basePosition.position.x = msg[1][0]
            self.basePosition.position.y = msg[1][1]
            self.basePosition.position.z = msg[1][2]

    def image_Callback(self, msg):
        print("---- image_Callback not realised", msg)

    def geGyroX_Callback(self, msg):
        """ Получение знач. с гироскопа (угловая скорость, град/сек ? рад/сек) """
        if (msg[0] == True): self.gyro.x = msg[1]

    def geGyroY_Callback(self, msg):
        """ Получение знач. с гироскопа (угловая скорость, град/сек ? рад/сек) """
        if (msg[0] == True): self.gyro.y = msg[1]

    def geGyroZ_Callback(self, msg):
        """ Получение знач. с гироскопа (угловая скорость, град/сек ? рад/сек) """
        if (msg[0] == True): self.gyro.z = msg[1]

    def geAccelX_Callback(self, msg):
        """ Получение знач. с акселерометра (линейное ускорение, м/сек^2) """
        if msg[1] != None:
            self.accel.x = msg[1]
        else:
            self.accel.x = 0
        if self.isVerbose: print("geAccelX ", msg[1], self.accel.x)

    def geAccelY_Callback(self, msg):
        """ Получение знач. с акселерометра (линейное ускорение, м/сек^2) """
        if msg[1] != None:
            self.accel.y = msg[1]
        else:
            self.accel.y = 0
        if self.isVerbose: print("geAccelY ", msg[1], self.accel.y)

    def geAccelZ_Callback(self, msg):
        """ Получение знач. с акселерометра (линейное ускорение, м/сек^2) """
        if msg[1] != None:
            self.accel.z = msg[1]
        else:
            self.accel.z = 0
        if self.isVerbose: print("geAccelZ ", msg[1], self.accel.z)

    def visionReadCallbackL(self, msg):
        """ Извлечь среднюю яркость кадра визуального датчика, диапазон [0,1.0] """
        if (msg[0] and len(msg) == 3):
            self.sensVisL = msg[2][10]
            if self.isVerbose: print('    visL Avg intensivity: ', self.sensVisL)

    def visionReadCallbackR(self, msg):
        """ Извлечь среднюю яркость кадра визуального датчика, диапазон [0,1.0] """
        if (msg[0] and len(msg) == 3):
            self.sensVisR = msg[2][10]
            if self.isVerbose: print('    visR Avg intensivity: ', self.sensVisR)

    def ultrasonicReadCallBack(self, msg):
        """ Опрос датчка расстояния. Вывод только в случае обнаружения препятствия, при отсутствии 2.25 метра """
        '''
        A list that contains:
            item1 (bool):  Whether the function was successfully called on the server side
            item2 (number): detection state (0 or 1)
            item3 (number): The distance to the detected point
            item4 (list):   The detected point relative to the sensor frame
            item5 (number): The detected object handle
            item6 (list):   The normal vector of the detected surface, relative to the sensor frame
        '''

        isNotError = msg[0]
        isDetected = msg[1]
        if (isNotError and isDetected == 1):
            self.sensDistance_F = msg[2]
            if self.isVerbose:
                print('    prox_dist_F {:.3f}'.format(self.sensDistance_F))
        else:
            self.sensDistance_F = 2.25



    def ultrasonicReadCallBack_2(self, msg):
        """ Опрос датчка расстояния. Вывод только в случае обнаружения препятствия, при отсутствии 2.25 метра """
        '''
        A list that contains:
            item1 (bool):  Whether the function was successfully called on the server side
            item2 (number): detection state (0 or 1)
            item3 (number): The distance to the detected point
            item4 (list):   The detected point relative to the sensor frame
            item5 (number): The detected object handle
            item6 (list):   The normal vector of the detected surface, relative to the sensor frame
        '''

        isNotError = msg[0]
        isDetected = msg[1]
        if (isNotError and isDetected == 1):
            self.sensDistance_S = msg[2]
            if self.isVerbose:
                print('    prox_dist_S          {:.3f}'.format(self.sensDistance_S))
        else:
            self.sensDistance_S = 2.25

    # -------------------------------------------------------------------------------------------------------------------------------

    def calcFullRotateAngle(self, pos_deg, pos_deg_Prev):
        """
        Вычисление полного угла поворота с учетом перехода через 0 отметку
        :param pos_deg:       положение текущее, [град]
        :param pos_deg_Prev:  положение предыдущее, [град]
        :return:              полный угол поворота, [град]
        """
        diffDegree = pos_deg - pos_deg_Prev

        if diffDegree >= -360 and diffDegree <= -180:
            # переход через 0 при провороте в сторону УВЕЛИЧЕНИЯ градусов
            under = 360 - pos_deg_Prev  # неучтенный остаток pos_deg_Prev
            if self.isVerbose: print("   Колесо. Переход через 0 уВЕЛИЧЕния. Угол: остаток {:3.4f}, полный {:3.4f}".format(
                under, pos_deg + under))
            return pos_deg + under

        elif diffDegree >= 180 and diffDegree <= 360:
            # переход через 0 при провороте в сторону УМЕНЬШЕНИЯ градусов
            under = 360 - pos_deg  # неучтенный остаток pos_deg
            if self.isVerbose: print("   Колесо. Переход через 0 уМЕНЬШЕния. Угол: остаток {:3.4f}, полный {:3.4f}".format(
                under, pos_deg_Prev + under))
            return pos_deg_Prev + under

        else:
            # поворот в пределах 0...360 без пересечения нуля
            return diffDegree


    def revJ_wheelL_ReadAngle_Callback(self, msg):
        """
        Получить положение угла поворота мотора/колеса
        # поворотное циклическое соединение (revolute joint)
        #   изменяет значения радианы {0 ... +3.14 ... -3.14 ... 0}
        #   изменяет значения градусы 0 +1 ... +179 +180 | -180 -179 ... -1 0
        """
        self.revJ_wheelL_pos_rad_Prev = self.revJ_wheelL_pos_rad
        self.revJ_wheelL_pos_deg_Prev = self.revJ_wheelL_pos_deg
        self.revJ_wheelL_pos_rad = msg[1]

        # Перевести радианы  {0 ... +3.14 ... -3.14 ... 0} в градусы {0 ... 360}
        #  0 ... 3.14 => 0...180
        # -3.14 ... 0 => 181...360
        self.revJ_wheelL_pos_deg = math.degrees(msg[1]) + (0 if msg[1] >= 0 else 360)  # скобки обязательны

        # вычислить полный прирост угла поворота
        self.revJ_wheelL_pos_deg_Delta = self.calcFullRotateAngle(self.revJ_wheelL_pos_deg, self.revJ_wheelL_pos_deg_Prev)
        self.revJ_wheelL_pos_deg_Total += self.revJ_wheelL_pos_deg_Delta
        #print("revJ_wheelL_ReadAngle_Callback", self.revJ_wheelL_pos_deg_Total, self.revJ_wheelL_pos_deg)





    def revJ_wheelR_ReadAngle_Callback(self, msg):
        """
        Получить положение угла поворота мотора/колеса
        # поворотное циклическое соединение (revolute joint)
        #   изменяет значения радианы {0 ... +3.14 ... -3.14 ... 0}
        #   изменяет значения градусы 0 +1 ... +179 +180 | -180 -179 ... -1 0
        """

        self.revJ_wheelR_pos_rad_Prev = self.revJ_wheelR_pos_rad
        self.revJ_wheelR_pos_deg_Prev = self.revJ_wheelR_pos_deg
        self.revJ_wheelR_pos_rad = msg[1]

        # Перевести радианы  {0 ... +3.14 ... -3.14 ... 0} в градусы {0 ... 360}
        #  0 ... 3.14 => 0...180
        # -3.14 ... 0 => 181...360
        self.revJ_wheelR_pos_deg = math.degrees(msg[1]) + (0 if msg[1] >= 0 else 360) # скобки обязательны

        # вычислить полный прирост угла поворота
        self.revJ_wheelR_pos_deg_Delta = self.calcFullRotateAngle(self.revJ_wheelR_pos_deg, self.revJ_wheelR_pos_deg_Prev)
        self.revJ_wheelR_pos_deg_Total += self.revJ_wheelR_pos_deg_Delta
        ##print(" revJ_wheelR_ReadAngle_Callback Тек {:4.3f}; Пред{:4.3f}; Разница {:4.3f}; Накопл {:4.3f}".format(
        #    self.revJ_wheelR_pos_deg, self.revJ_wheelR_pos_deg_Prev,
        #    self.revJ_wheelR_pos_deg-self.revJ_wheelR_pos_deg_Prev,
        #    self.revJ_wheelR_pos_Deg_Total
        # ))


    def revJ_sensor_ReadAngle_Callback(self, msg):
        """
        Угол поворота датчика расстояния
        :param msg:
        :return:
        """
        self.revJ_sensor_pos_rad_ABS = msg[1]

        # Перевести радианы  {0 ... +3.14 ... -3.14 ... 0} в градусы {0 ... 360}
        #  0 ... 3.14 => 0...180
        # -3.14 ... 0 => 181...360
        self.revJ_sensor_pos_deg_ABS = math.degrees(msg[1]) + 0 if msg[1] >= 0 else 360
        pass

    #########################################################################################
    #  Управление ходовой частью
    #########################################################################################
    def setStopWheel(self):
        """
        Остановить ходовые моторы
        :return:
        """
        self.setTankSpeed(0, 0)



    def setTankSpeed(self, powL, powR):
        """
        Танковое управление двумя моторами. Принимает мощность моторов {-1...1}
        # Особенность simxSetJointTargetVelocity: требуется передавать значение циклической частоты.
        :param powL: -1.0 ..0.. +1.0 мощность мотора от self.wheelSpeed_L_Rad
        :param powR: -1.0 ..0.. +1.0 мощность мотора от self.wheelSpeed_L_Rad
        :return:
        """
        ##print("setTankSpeed")

        # Проверка граничных значений
        powL = -1.0 if powL < -1.0 else 1.0 if powL > 1.0 else powL
        powR = -1.0 if powR < -1.0 else 1.0 if powR > 1.0 else powR

        self.wheelSpeed_L_Rad = self.maxSpeedWheel_Rad * powL
        self.wheelSpeed_R_Rad = self.maxSpeedWheel_Rad * powR

        self.client.simxSetJointTargetVelocity(self.revJ_wheelL_H, self.wheelSpeed_L_Rad, self.client.simxDefaultPublisher())
        self.client.simxSetJointTargetVelocity(self.revJ_wheelR_H, self.wheelSpeed_R_Rad, self.client.simxDefaultPublisher())

        self.calcVelocity()
        ####self.printCurrSpeed()


    def setSteerSpeed(self, direction, powSpeed):
        """
        Рулевое управление двумя моторами.
        :param direction: Направление в диапазоне от  -1.0 ... 0 ... +1.0, где: |1.0|- разворот на месте; |0.5| поворот вокруг колеса
        :param powSpeed: -1.0 ... +1.0 мощность мотора от self.maxSpeedWheel_degrePerSec
        :return:
        """
        # Проверка граничных значений
        powSpeed = -1.0 if powSpeed < -1.0 else 1.0 if powSpeed > 1.0 else powSpeed

        if (direction == 0):
            self.setTankSpeed(powSpeed, powSpeed)
        else:
            #  y(x)= -2 * x
            if direction > 1: direction = 1
            if direction < -1: direction = -1
            speedReduceVal = powSpeed * ((abs(direction) * -2))

            if (direction > 0):
                self.setTankSpeed(powSpeed, powSpeed + speedReduceVal)  # reduce right
            else:
                self.setTankSpeed(powSpeed + speedReduceVal, powSpeed)  # reduce left

            if self.isVerbose:  print ("direction {:>2.2f} speedReduceVal {:>2.2f} - speed {:>2.2f} ".format(direction , speedReduceVal, powSpeed ))


    def calcVelocity(self):
        """
        Расчет скорости движения на основе цикл. скорости вращения колес
        :return:
        """
        ###wR = (2*vRobot + self.L*wRobot)/self.wheelDiametr # циклическая скорость правого колеса
        ###wL = (2*vRobot - self.L*wRobot)/self.wheelDiametr # циклическая скорость легово колеса
        ###self.wL = self.wheelSpeed_L_Rad     # циклическая скорость правого колеса, рад/сек
        ###self.wR = self.wheelSpeed_R_Rad     # циклическая скорость левого колеса, рад/сек
        dt = self.simTime - self.simTimePrev
        ####print("calcVelocity", dt)

        dwL = self.revJ_wheelL_pos_rad - self.revJ_wheelL_pos_rad_Prev
        dwR = self.revJ_wheelR_pos_rad - self.revJ_wheelR_pos_rad_Prev

        self.revJ_wheelL_pos_rad_Total += dwL
        self.revJ_wheelR_pos_rad_Total += dwR

        # Обратный ход. переход с -3.14 к +3.14
        if self.revJ_wheelL_pos_rad > 0 and self.revJ_wheelL_pos_rad_Prev < 0:
            dwL = abs(math.pi - self.revJ_wheelL_pos_rad) + abs(-math.pi - self.revJ_wheelL_pos_rad_Prev)
        if self.revJ_wheelR_pos_rad > 0 and self.revJ_wheelR_pos_rad_Prev < 0:
            dwR = abs(math.pi - self.revJ_wheelR_pos_rad) + abs(-math.pi - self.revJ_wheelR_pos_rad_Prev)

        # Прямой ход. переход с +3.14 к - 3.14
        if self.revJ_wheelL_pos_rad < 0 and self.revJ_wheelL_pos_rad_Prev > 0:
            dwL = abs(-math.pi - self.revJ_wheelL_pos_rad) + abs(math.pi - self.revJ_wheelL_pos_rad_Prev)
        if self.revJ_wheelR_pos_rad < 0 and self.revJ_wheelR_pos_rad_Prev > 0:
            dwR = abs(-math.pi - self.revJ_wheelR_pos_rad) + abs(math.pi - self.revJ_wheelR_pos_rad_Prev)

        # print("==== calcVelocity l{:+3.3f} r{:+3.3f} | {:+3.3f} {:+3.3f} | {:+3.3f} {:+3.3f} ".format(
        #    dwL, dwR,
        #    self.revJ_wheelL_pos_rad, self.revJ_wheelL_pos_rad_Prev,
        #    self.revJ_wheelR_pos_rad, self.revJ_wheelR_pos_rad_Prev,
        # ))

        if (dt > 0):
            self.wL = dwL / dt  # циклическая скорость левого колеса, рад/сек
            self.wR = dwR / dt  # циклическая скорость правого колеса, рад/сек
            self.vL = (self.wL * self.wheelDiametr) / 2  # линейная скорость колеса, м/сек
            self.vR = (self.wR * self.wheelDiametr) / 2  # линейная скорость колеса, м/сек
            self.vRobot = (self.vR + self.vL) / 2  # линейная скорость модели, м/сек
            self.wRobot = (self.vR - self.vL) / self.L  # циклическая скорость модели (поворот вокруг центральной оси), рад/сек


    def printCurrSpeed(self):
        """
        Вывод параметров скорости
        :return:
        """
        print("Текущая скорость:  \n  wL={:.4f} рад/с, wR={:.4f}рад/с  \n  vL={:.4f} м/с, vR={:.4f}м/с  \n  vRobot={:.4f}м/с,  wRobot={:.4f} рад/с".format(
            self.wL, self.wR,
            self.vL, self.vR,
            self.vRobot, self.wRobot,
        ))

    #########################################################################################
    #  Запуск/остановка симуляции
    #########################################################################################


    def simulationStepStarted(self, msg):
        """ Действия при начала кадра симуляции """
        self.simTime = msg[1][b'simulationTime']
        if self.isVerbose:
            print('Simulation step STARTED. SIM time: {:<5.3f}. REAL time: {:>5.3f} $ {:>5.3f} {:>5.3f} '.format(
                self.simTime, time.time() - self.realStartTime,
                self.revJ_wheelL_pos_deg_Total, self.revJ_wheelL_pos_deg))


    def simulationStepDone(self, msg):
        """   Действия при приостановке кадра симуляции """
        self.simTime = msg[1][b'simulationTime']
        if self.isVerbose:
            print('Simulation step DONE.    SIM time: {:<5.3f}. REAL time: {:>5.3f} $ {:>5.3f} {:>5.3f} '.format(
                self.simTime, time.time() - self.realStartTime,
                self.revJ_wheelL_pos_deg_Total, self.revJ_wheelL_pos_deg))
        self.client.doNextStep = True


    def startSim(self):
        """ Запустить симуляцию, ожидать отдельной команды выполнения кадров времени (в синхронном режиме)"""

        self.stopSim() # Остановить предыдущую симуляцию
        time.sleep(0.1)  # необходимая пауза для начальной расстановки объектов

        # Запустить симуляцию в синхронном режиме
        if self.client.runInSynchMode:
            self.client.simxSynchronous(True)
            print("> starttSim (simxSYNChronous)")
        else:
            print("> starttSim (simxASYNCchronous)")

        self.client.simxStartSimulation(self.client.simxDefaultPublisher())
        self.realStartTime = time.time()
        pass


    def stopSim(self):
        """ Остановить симуляцию """
        print("> stoptSim")
        self.setTankSpeed(0, 0)  # остановить робота
        self.client.simxStopSimulation(self.client.simxDefaultPublisher())


    # -------------------------------------------------------------------------------------------------------------------------------
    #########################################################################################
    #  Вспомогательные
    #########################################################################################
    def simWait(self, sec):
        """
        Ожидание внутри симуляции, проигрывает заданное количество секунд, но не изменяет параметры
        :param sec: время ожидания, сек
        :return:
        """
        simStartTime = self.simTime
        while self.simTime < simStartTime + sec:
            self.stepSimulationInternal()
        pass


    def stepSimulationInternal(self):
        """
        Функция обработки вычислений между кадрами симуляции
        """
        if self.client.runInSynchMode:
            while not self.client.doNextStep:
                self.client.simxSpinOnce()  # Передать значения.  отрабатывают все CallBack
            # Приостанавливает симуляцию. Доступно новое состояние
            self.client.doNextStep = False
            ############################################################
            self.client.simxSynchronousTrigger()  # Продолжает симуляцию на dt=50 мс (по умолчанию) . Движение сцены
            pass
        else:
            self.client.simxSpinOnce()  # Передать значения.  отрабатывают все CallBack


    # -------------------------------------------------------------------------------------------------------------------------------
    #########################################################################################
    #
    #########################################################################################
    def rotateSensorToABSPos(self, targetPosDegreeABS, speedDegPerSec):
        """
        Развернуть датчик расстояния в абсолютное целевое положение с заданной скоростью
        :param targetPosDegreeABS: Целевое положение датчика. Абсолютное [0, 360]
        :param speedDegPerSec:  Скорость вращения датчика
        :return:


        @@@todo: исправить знак градусов. тек.: против часовой
        @@@todo: проверка граничных условий
        """
        # Вычисление отклонения от целевого
        calcErrorAbs = abs(self.revJ_sensor_pos_deg_ABS - targetPosDegreeABS)

        # Начать вращение Вращение
        #self.client.simxSetJointTargetVelocity(self.revJ_sensor_H_1, math.radians(speedDegPerSec), self.client.simxDefaultPublisher())
        radPerSec= math.radians(speedDegPerSec)


        while calcErrorAbs > 0.1:
            # Вращать до тех пор пока отклонение не станет меньше допустимого


            if self.client.runInSynchMode:
                while not self.client.doNextStep:
                    self.client.simxSpinOnce()
                self.client.doNextStep = False

                ###########
                # Вращение
                if self.isVerbose:
                    print("rotateSensorToPos", self.revJ_sensor_pos_deg_ABS, calcErrorAbs)


                calcErrorAbs = abs(self.revJ_sensor_pos_deg_ABS - targetPosDegreeABS)


                speeddd = radPerSec * ((calcErrorAbs/targetPosDegreeABS)*10)

                self.client.simxSetJointTargetVelocity(self.revJ_sensor_H_1,   speeddd, self.client.simxDefaultPublisher())
                ###########

                self.client.simxSynchronousTrigger()
            else:
                self.client.simxSpinOnce()
            pass

        # Остановить вращение
        self.client.simxSetJointTargetVelocity(self.revJ_sensor_H_1, 0, self.client.simxDefaultPublisher())
        pass







    def rotateWheel(self, direction, powSpeed, relativeRotateAngle_deg):
        """
        @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        :param direction: Направление в диапазоне от  -1.0 ... 0 ... +1.0, где: |1.0|- разворот на месте; |0.5| поворот вокруг колеса
        :param powSpeed: -1.0 ... +1.0 мощность мотора от self.maxSpeedWheel_degrePerSec
        :param relativeRotateAngle_deg: Только ПОЛОЖИТЕЛЬНЫЙ, расчет в указанную сторону
        :return:
        """
        """     
        powSpeed +,   relativeRotateAngle_deg +   вперед
        powSpeed -,   relativeRotateAngle_deg +   назад
        """


        relativeRotateAngle_deg =  abs(relativeRotateAngle_deg)

        if powSpeed == 0 or relativeRotateAngle_deg == 0: return

        if powSpeed > 0:
            TotalTarget_wheelL_pos_deg = self.revJ_wheelL_pos_deg_Total + relativeRotateAngle_deg
            TotalTarget_wheelR_pos_deg = self.revJ_wheelR_pos_deg_Total + relativeRotateAngle_deg
        else:
            TotalTarget_wheelL_pos_deg = self.revJ_wheelL_pos_deg_Total - relativeRotateAngle_deg
            TotalTarget_wheelR_pos_deg = self.revJ_wheelR_pos_deg_Total - relativeRotateAngle_deg


        calcErrorLeftABS_pos_Deg_Total  = abs(TotalTarget_wheelL_pos_deg - self.revJ_wheelL_pos_deg_Total)
        calcErrorRightABS_pos_Deg_Total = abs(TotalTarget_wheelR_pos_deg - self.revJ_wheelR_pos_deg_Total)


        while calcErrorLeftABS_pos_Deg_Total > 0.5:   #and  calcErrorRightABS_pos_Deg_Total > 0.1:
            # Вращать до тех пор пока отклонение не станет меньше допустимого

            if self.client.runInSynchMode:
                while not self.client.doNextStep:
                    self.client.simxSpinOnce()

                self.client.doNextStep = False

                ########################################################################################
                # Вращение
                ###########



                #wheelLeftABS_pos_Deg_delta = abs(TotalTarget_wheelL_pos_deg -  self.revJ_wheelL_pos_deg_Total )

                calcErrorLeftABS_pos_Deg_Total = abs(TotalTarget_wheelL_pos_deg - self.revJ_wheelL_pos_deg_Total)
                calcErrorRightABS_pos_Deg_Total = abs(TotalTarget_wheelR_pos_deg - self.revJ_wheelR_pos_deg_Total)

                ## todo взять скорость в градусах
                # получить Макс. прирост угла поворота из двух колес
                thresshold = abs(max(self.revJ_wheelL_pos_deg_Delta, self.revJ_wheelR_pos_deg_Delta))

                # если остаток до целевого меньше, следующего приращения => проскочит
                if calcErrorLeftABS_pos_Deg_Total > thresshold*4:
                    kProp = 1
                else:
                    #тогда использовать пропорциональное замедление
                    kProp = calcErrorLeftABS_pos_Deg_Total/(thresshold*1)


                self.setSteerSpeed(direction, powSpeed*kProp)
                ########################################################################################
                self.client.simxSynchronousTrigger()


                sensVisLprint("4 rotateSensorToPos {:2.2f}  {:2.3f}  {:2.3f} $ {:>5.3f} {:>5.3f}".format(
                    self.revJ_sensor_pos_deg_ABS, calcErrorLeftABS_pos_Deg_Total,
                    0,  # calcErrorRightABS_pos_Deg_Total,
                    self.revJ_wheelL_pos_deg_Total,
                    self.revJ_wheelL_pos_deg))
            else:
                self.client.simxSpinOnce()
        pass


        # Остановить колеса
        self.setStopWheel()
        pass



    # -------------------------------------------------------------------------------------------------------------------------------
    #########################################################################################
    #
    #########################################################################################

    def deldel(self):
        print("deldel")
        pass

#########################################################################################
# Main function for testing
#########################################################################################
if __name__ == "__main__":
    # Подключение к модели робота
    # todo: см. прием вызова из внешнего файла
    pass
