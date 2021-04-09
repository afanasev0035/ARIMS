from VehicleDHAL import VehicleDHAL
import time

def PD(robotSim, prevErr):
    err = robotSim.sensVisL - robotSim.sensVisR
    if (prevErr > 0.7):
        black_white = 1
        prevErr = 0
    else:
        black_white = 0
        prevErr = 0
    if black_white == 0: err *= (-1)
    P = 2
    ControlSignal = err * P
    D = 1.5
    DiffSignal = (err - prevErr) * D
    PD = ControlSignal + DiffSignal
    return err


def direct(robotSim):
    
   
    robotSim.setSteerSpeed(direction=0, powSpeed=0.2)
    robotSim.simWait(2.5)  # выполнить несколько сек. кадров времени симуляции
    print("Еду  прямо simTime={:>2.2f};  sensDistance_S={:>2.2f};  sensDistance_F={:>2.2f}".format(
        robotSim.simTime, robotSim.sensDistance_S, robotSim.sensDistance_F))
    pass

def right(robotSim):
    # -- Поворот направо.Установить скорости вращения колес
    robotSim.setSteerSpeed(direction=0.8, powSpeed=0.2)
    robotSim.simWait(1.95)  # выполнить несколько сек. кадров времени симуляции
    print("Еду направо simTime={:>2.2f};  sensDistance_S={:>2.2f};  sensDistance_F={:>2.2f}".format(
        robotSim.simTime, robotSim.sensDistance_S, robotSim.sensDistance_F))
    pass

def left(robotSim):
    # -- Поворот налево. Установить скорости вращения колес
    robotSim.setSteerSpeed(direction=-0.80, powSpeed=0.2)
    robotSim.simWait(1.95)  # выполнить несколько сек. кадров времени симуляции
    print("Еду налево simTime={:>2.2f};  sensDistance_S={:>2.2f};  sensDistance_F={:>2.2f}".format(
        robotSim.simTime, robotSim.sensDistance_S, robotSim.sensDistance_F))
    pass
# -------------------------------------------------------------------------------------
if __name__ == "__main__":

    # Создать объект-обертку для робота Vehicle
    robotSim = VehicleDHAL(
        b0ChannelName="b0RemoteApi",  # Имя канала b0RemoteApi Обязательный параметр
        maxSpeedWheel_meterPerSec=0.4,  # макс. допустимая скорость, [м/с]
        isVisSensorActive=True,  # Флаг акт. визуальных датчиков, откл. для быстрой инициализ.
        isProxSensorActive=True,  # Флаг акт. датчика расстояния, откл. для быстрой инициализ.
        isAccelActive=False,  # Флаг акт.акселерометра, откл. для быстрой инициализ.
        isGyroActive=False  # Флаг акт.гироскопа, откл. для быстрой инициализ.
    )
    
    # Запустить симуляцию, но ожидать отдельной команды выполнения кадров времени симуляции
    robotSim.startSim()
    route = 2
    # Главный цикл
    simStartTime = robotSim.simTime
    global e
    e = 0
    robotSim.simWait(2)
    # robotSim.simTime, robotSim.sensVisL, robotSim.sensDistance_F
    while robotSim.simTime < simStartTime + 400:  # в течении __ секунд кадров симуляции
        
        print("simTime={:>2.2f};  sensDistance_S={:>2.2f};  sensDistance_F={:>2.2f}; sensVisL={:>2.2f}; sensVisR={:>2.2f}".format(
            robotSim.simTime, robotSim.sensDistance_S, robotSim.sensDistance_F, robotSim.sensVisL, robotSim.sensVisR))

        if ((robotSim.sensVisL > 0.65 and robotSim.sensVisL  < 0.7) and (robotSim.sensVisR > 0.65 and robotSim.sensVisR  < 0.7)):
            direct(robotSim)
            print("Я приехаль, товарищъ")
            break

        if (robotSim.sensDistance_S > 0.25):
            left(robotSim)
            direct(robotSim)
            direct(robotSim)
        elif (robotSim.sensDistance_F < 0.25):
            right(robotSim)
            if (robotSim.sensDistance_F < 0.25):
                right(robotSim)
            direct(robotSim)
        else:
            direct(robotSim)
            PD(robotSim, e)
        
        
        
        

    # Выждать __ реальных секунд
    time.sleep(3)

    # Остановить симуляцию
    robotSim.stopSim()