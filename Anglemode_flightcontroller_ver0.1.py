# MicroPython port that preserves original function and variable names
# Original: Anglemode_flightcontroller_ver3.1.ino

from machine import Pin, I2C
import utime
import math
from ESP32_Servo import Servo

# preserve global variables and their names
RatePitch = 0.0
RateRoll = 0.0
RateYaw = 0.0
RateCalibrationPitch = 0.0
RateCalibrationRoll = 0.0
RateCalibrationYaw = 0.0
AccXCalibration = 0.0
AccYCalibration = 0.0
AccZCalibration = 0.0

ESCfreq = 500
PAngleRoll = 2; PAnglePitch = PAngleRoll
IAngleRoll = 0.5; IAnglePitch = IAngleRoll
DAngleRoll = 0.007; DAnglePitch = DAngleRoll

PRateRoll = 0.625
IRateRoll = 2.1
DRateRoll = 0.0088

PRatePitch = PRateRoll
IRatePitch = IRateRoll
DRatePitch = DRateRoll

PRateYaw = 4
IRateYaw = 3
DRateYaw = 0

LoopTimer = 0
t = 0.004  # time cycle

mot1 = Servo()
mot2 = Servo()
mot3 = Servo()
mot4 = Servo()

mot1_pin = 13
mot2_pin = 12
mot3_pin = 14
mot4_pin = 27

current_time = 0
last_channel_1 = 0
last_channel_2 = 0
last_channel_3 = 0
last_channel_4 = 0
last_channel_5 = 0
last_channel_6 = 0
timer_1 = 0
timer_2 = 0
timer_3 = 0
timer_4 = 0
timer_5 = 0
timer_6 = 0
ReceiverValue = [0, 0, 0, 0, 0, 0]

channel_1_pin = 34
channel_2_pin = 35
channel_3_pin = 32
channel_4_pin = 33
channel_5_pin = 25
channel_6_pin = 26

# MicroPython Pin instances (created in setup)
_channel_1 = None
_channel_2 = None
_channel_3 = None
_channel_4 = None
_channel_5 = None
_channel_6 = None

PtermRoll = 0.0
ItermRoll = 0.0
DtermRoll = 0.0
PIDOutputRoll = 0.0
PtermPitch = 0.0
ItermPitch = 0.0
DtermPitch = 0.0
PIDOutputPitch = 0.0
PtermYaw = 0.0
ItermYaw = 0.0
DtermYaw = 0.0
PIDOutputYaw = 0.0
KalmanGainPitch = 0.0
KalmanGainRoll = 0.0

ThrottleIdle = 1170
ThrottleCutOff = 1000

DesiredRateRoll = 0.0
DesiredRatePitch = 0.0
DesiredRateYaw = 0.0
ErrorRateRoll = 0.0
ErrorRatePitch = 0.0
ErrorRateYaw = 0.0
InputRoll = 0.0
InputThrottle = 0.0
InputPitch = 0.0
InputYaw = 0.0
PrevErrorRateRoll = 0.0
PrevErrorRatePitch = 0.0
PrevErrorRateYaw = 0.0
PrevItermRateRoll = 0.0
PrevItermRatePitch = 0.0
PrevItermRateYaw = 0.0
PIDReturn = [0.0, 0.0, 0.0]

AccX = 0.0
AccY = 0.0
AccZ = 0.0
AngleRoll = 0.0
AnglePitch = 0.0
KalmanAngleRoll = 0.0
KalmanUncertaintyAngleRoll = 2 * 2
KalmanAnglePitch = 0.0
KalmanUncertaintyAnglePitch = 2 * 2
Kalman1DOutput = [0.0, 0.0]
DesiredAngleRoll = 0.0
DesiredAnglePitch = 0.0
ErrorAngleRoll = 0.0
ErrorAnglePitch = 0.0
PrevErrorAngleRoll = 0.0
PrevErrorAnglePitch = 0.0
PrevItermAngleRoll = 0.0
PrevItermAnglePitch = 0.0

complementaryAngleRoll = 0.0  # Changed from 0.0f to 0.0
complementaryAnglePitch = 0.0  # Changed from 0.0f to 0.0

MotorInput1 = 0.0
MotorInput2 = 0.0
MotorInput3 = 0.0
MotorInput4 = 0.0

# I2C device
_i2c = None
_MPU_ADDR = 0x68

def kalman_1d(KalmanState, KalmanUncertainty, KalmanInput, KalmanMeasurement):
    global Kalman1DOutput, KalmanGainRoll, KalmanGainPitch
    KalmanState = KalmanState + (t * KalmanInput)
    KalmanUncertainty = KalmanUncertainty + (t * t * 4 * 4)  # IMU variance 4 deg/s
    KalmanGain = KalmanUncertainty * 1 / (1 * KalmanUncertainty + 3 * 3)  # error variance 3 deg
    KalmanState = KalmanState + KalmanGain * (KalmanMeasurement - KalmanState)
    KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty
    Kalman1DOutput[0] = KalmanState
    Kalman1DOutput[1] = KalmanUncertainty

def _read16(bh, bl):
    # helper: combine two bytes to signed 16-bit
    val = (bh << 8) | bl
    if val & 0x8000:
        val = -((~val & 0xFFFF) + 1)
    return val

def gyro_signals():
    global AccX, AccY, AccZ, RateRoll, RatePitch, RateYaw, AngleRoll, AnglePitch
    # config registers
    _i2c.writeto_mem(_MPU_ADDR, 0x1A, bytes([0x03]))
    _i2c.writeto_mem(_MPU_ADDR, 0x1C, bytes([0x10]))
    # read accel
    acc = _i2c.readfrom_mem(_MPU_ADDR, 0x3B, 6)
    AccXLSB = _read16(acc[0], acc[1])
    AccYLSB = _read16(acc[2], acc[3])
    AccZLSB = _read16(acc[4], acc[5])
    # gyro config
    _i2c.writeto_mem(_MPU_ADDR, 0x1B, bytes([0x08]))
    # read gyro
    gyr = _i2c.readfrom_mem(_MPU_ADDR, 0x43, 6)
    GyroX = _read16(gyr[0], gyr[1])
    GyroY = _read16(gyr[2], gyr[3])
    GyroZ = _read16(gyr[4], gyr[5])

    RateRoll = GyroX / 65.5
    RatePitch = GyroY / 65.5
    RateYaw = GyroZ / 65.5
    AccX = AccXLSB / 4096.0
    AccY = AccYLSB / 4096.0
    AccZ = AccZLSB / 4096.0
    AngleRoll = math.atan(AccY / math.sqrt(AccX * AccX + AccZ * AccZ)) * 57.29
    AnglePitch = -math.atan(AccX / math.sqrt(AccY * AccY + AccZ * AccZ)) * 57.29

def pid_equation(Error, P, I, D, PrevError, PrevIterm):
    global PIDReturn
    Pterm = P * Error
    Iterm = PrevIterm + (I * (Error + PrevError) * (t / 2))
    if Iterm > 400:
        Iterm = 400
    elif Iterm < -400:
        Iterm = -400
    Dterm = D * ((Error - PrevError) / t)
    PIDOutput = Pterm + Iterm + Dterm
    if PIDOutput > 400:
        PIDOutput = 400
    elif PIDOutput < -400:
        PIDOutput = -400
    PIDReturn[0] = PIDOutput
    PIDReturn[1] = Error
    PIDReturn[2] = Iterm

def channelInterruptHandler(pin):
    # preserve name; MicroPython IRQ passes pin
    global current_time
    global last_channel_1, last_channel_2, last_channel_3, last_channel_4, last_channel_5, last_channel_6
    global timer_1, timer_2, timer_3, timer_4, timer_5, timer_6
    global ReceiverValue

    current_time = utime.ticks_us()
    v = pin.value()  # 1 rising, 0 falling
    # match pin to channel
    if pin == _channel_1:
        if v == 1:
            if last_channel_1 == 0:
                last_channel_1 = 1
                timer_1 = current_time
        elif last_channel_1 == 1:
            last_channel_1 = 0
            ReceiverValue[0] = utime.ticks_diff(current_time, timer_1)
    elif pin == _channel_2:
        if v == 1:
            if last_channel_2 == 0:
                last_channel_2 = 1
                timer_2 = current_time
        elif last_channel_2 == 1:
            last_channel_2 = 0
            ReceiverValue[1] = utime.ticks_diff(current_time, timer_2)
    elif pin == _channel_3:
        if v == 1:
            if last_channel_3 == 0:
                last_channel_3 = 1
                timer_3 = current_time
        elif last_channel_3 == 1:
            last_channel_3 = 0
            ReceiverValue[2] = utime.ticks_diff(current_time, timer_3)
    elif pin == _channel_4:
        if v == 1:
            if last_channel_4 == 0:
                last_channel_4 = 1
                timer_4 = current_time
        elif last_channel_4 == 1:
            last_channel_4 = 0
            ReceiverValue[3] = utime.ticks_diff(current_time, timer_4)
    elif pin == _channel_5:
        if v == 1:
            if last_channel_5 == 0:
                last_channel_5 = 1
                timer_5 = current_time
        elif last_channel_5 == 1:
            last_channel_5 = 0
            ReceiverValue[4] = utime.ticks_diff(current_time, timer_5)
    elif pin == _channel_6:
        if v == 1:
            if last_channel_6 == 0:
                last_channel_6 = 1
                timer_6 = current_time
        elif last_channel_6 == 1:
            last_channel_6 = 0
            ReceiverValue[5] = utime.ticks_diff(current_time, timer_6)

def setup():
    global LoopTimer, _i2c
    global _channel_1, _channel_2, _channel_3, _channel_4, _channel_5, _channel_6
    
    # Serial initialization (print statement for MicroPython)
    print("Starting flight controller...")
    
    # LED blink sequence - match C++ version exactly (9 blinks)
    led_time = 100  # 100ms like C++ version
    led = Pin(15, Pin.OUT)
    
    # First 8 blinks (4 cycles of on-off)
    for _ in range(4):
        led.value(0); utime.sleep_ms(led_time)
        led.value(1); utime.sleep_ms(led_time)
        led.value(0); utime.sleep_ms(led_time)
        led.value(1); utime.sleep_ms(led_time)
    
    # Final blink
    led.value(0); utime.sleep_ms(led_time)

    # Receiver pins with pull-up and IRQ on both edges
    _channel_1 = Pin(channel_1_pin, Pin.IN, Pin.PULL_UP)
    _channel_2 = Pin(channel_2_pin, Pin.IN, Pin.PULL_UP)
    _channel_3 = Pin(channel_3_pin, Pin.IN, Pin.PULL_UP)
    _channel_4 = Pin(channel_4_pin, Pin.IN, Pin.PULL_UP)
    _channel_5 = Pin(channel_5_pin, Pin.IN, Pin.PULL_UP)
    _channel_6 = Pin(channel_6_pin, Pin.IN, Pin.PULL_UP)

    _channel_1.irq(handler=channelInterruptHandler, trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING)
    _channel_2.irq(handler=channelInterruptHandler, trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING)
    _channel_3.irq(handler=channelInterruptHandler, trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING)
    _channel_4.irq(handler=channelInterruptHandler, trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING)
    _channel_5.irq(handler=channelInterruptHandler, trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING)
    _channel_6.irq(handler=channelInterruptHandler, trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING)

    utime.sleep_ms(100)

    # I2C init (ESP32 default pins SDA=21, SCL=22) with 400kHz like C++ version
    _i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)
    utime.sleep_ms(250)
    # wake MPU6050
    _i2c.writeto_mem(_MPU_ADDR, 0x6B, bytes([0x00]))

    # Note: ESP32PWM timer allocation is handled automatically in MicroPython
    # The C++ version calls ESP32PWM::allocateTimer(0-3) but this is not needed in MicroPython

    # Motors attach and set ESC frequency - match C++ timing exactly
    utime.sleep_ms(1000)
    mot1.attach(mot1_pin, 1000, 2000); utime.sleep_ms(1000); mot1.setPeriodHertz(ESCfreq); utime.sleep_ms(100)
    mot2.attach(mot2_pin, 1000, 2000); utime.sleep_ms(1000); mot2.setPeriodHertz(ESCfreq); utime.sleep_ms(100)
    mot3.attach(mot3_pin, 1000, 2000); utime.sleep_ms(1000); mot3.setPeriodHertz(ESCfreq); utime.sleep_ms(100)
    mot4.attach(mot4_pin, 1000, 2000); utime.sleep_ms(1000); mot4.setPeriodHertz(ESCfreq); utime.sleep_ms(100)

    mot1.writeMicroseconds(1000)
    mot2.writeMicroseconds(1000)
    mot3.writeMicroseconds(1000)
    mot4.writeMicroseconds(1000)
    utime.sleep_ms(500)
    
    # Final LED sequence to match C++ version
    led.value(0)
    led.value(1)
    utime.sleep_ms(500)
    led.value(0)
    utime.sleep_ms(500)

    # Calibrations - match C++ values exactly
    global RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw
    global AccXCalibration, AccYCalibration, AccZCalibration
    RateCalibrationRoll = 0.27
    RateCalibrationPitch = -0.85
    RateCalibrationYaw = -2.09
    AccXCalibration = 0.03
    AccYCalibration = 0.01
    AccZCalibration = -0.07

    LoopTimer = utime.ticks_us()

def loop():
    global RateRoll, RatePitch, RateYaw
    global AccX, AccY, AccZ
    global AngleRoll, AnglePitch
    global complementaryAngleRoll, complementaryAnglePitch
    global DesiredAngleRoll, DesiredAnglePitch, InputThrottle, DesiredRateYaw
    global ErrorAngleRoll, ErrorAnglePitch
    global PtermRoll, ItermRoll, DtermRoll, PIDOutputRoll
    global PtermPitch, ItermPitch, DtermPitch, PIDOutputPitch
    global PrevErrorAngleRoll, PrevErrorAnglePitch, PrevItermAngleRoll, PrevItermAnglePitch
    global ErrorRateRoll, ErrorRatePitch, ErrorRateYaw
    global PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw
    global PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw
    global InputRoll, InputPitch, InputYaw
    global MotorInput1, MotorInput2, MotorInput3, MotorInput4
    global LoopTimer

    # IMU config and read (preserve original per-loop config)
    # Match C++ version exactly - use 0x05 for DLPF in loop (not 0x03 like gyro_signals)
    _i2c.writeto_mem(_MPU_ADDR, 0x1A, bytes([0x05]))
    _i2c.writeto_mem(_MPU_ADDR, 0x1C, bytes([0x10]))
    acc = _i2c.readfrom_mem(_MPU_ADDR, 0x3B, 6)
    AccXLSB = _read16(acc[0], acc[1])
    AccYLSB = _read16(acc[2], acc[3])
    AccZLSB = _read16(acc[4], acc[5])
    _i2c.writeto_mem(_MPU_ADDR, 0x1B, bytes([0x08]))
    gyr = _i2c.readfrom_mem(_MPU_ADDR, 0x43, 6)
    GyroX = _read16(gyr[0], gyr[1])
    GyroY = _read16(gyr[2], gyr[3])
    GyroZ = _read16(gyr[4], gyr[5])

    RateRoll = GyroX / 65.5
    RatePitch = GyroY / 65.5
    RateYaw = GyroZ / 65.5
    AccX = AccXLSB / 4096.0
    AccY = AccYLSB / 4096.0
    AccZ = AccZLSB / 4096.0

    # apply calibrations
    RateRoll -= RateCalibrationRoll
    RatePitch -= RateCalibrationPitch
    RateYaw -= RateCalibrationYaw

    AccX -= AccXCalibration
    AccY -= AccYCalibration
    AccZ -= AccZCalibration

    AngleRoll = math.atan(AccY / math.sqrt(AccX * AccX + AccZ * AccZ)) * 57.29
    AnglePitch = -math.atan(AccX / math.sqrt(AccY * AccY + AccZ * AccZ)) * 57.29

    # complementary filter
    complementaryAngleRoll = 0.991 * (complementaryAngleRoll + RateRoll * t) + 0.009 * AngleRoll
    complementaryAnglePitch = 0.991 * (complementaryAnglePitch + RatePitch * t) + 0.009 * AnglePitch
    complementaryAngleRoll = 20 if complementaryAngleRoll > 20 else (-20 if complementaryAngleRoll < -20 else complementaryAngleRoll)
    complementaryAnglePitch = 20 if complementaryAnglePitch > 20 else (-20 if complementaryAnglePitch < -20 else complementaryAnglePitch)

    # receiver mapping
    DesiredAngleRoll = 0.1 * (ReceiverValue[0] - 1500)
    DesiredAnglePitch = 0.1 * (ReceiverValue[1] - 1500)
    InputThrottle = ReceiverValue[2]
    DesiredRateYaw = 0.15 * (ReceiverValue[3] - 1500)

    # angle mode PID -> rate demand
    ErrorAngleRoll = DesiredAngleRoll - complementaryAngleRoll
    PtermRoll = PAngleRoll * ErrorAngleRoll
    ItermRoll = PrevItermAngleRoll + (IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * (t / 2))
    ItermRoll = 400 if ItermRoll > 400 else (-400 if ItermRoll < -400 else ItermRoll)
    DtermRoll = DAngleRoll * ((ErrorAngleRoll - PrevErrorAngleRoll) / t)
    PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll
    PIDOutputRoll = 400 if PIDOutputRoll > 400 else (-400 if PIDOutputRoll < -400 else PIDOutputRoll)
    DesiredRateRoll = PIDOutputRoll
    PrevErrorAngleRoll = ErrorAngleRoll
    PrevItermAngleRoll = ItermRoll

    ErrorAnglePitch = DesiredAnglePitch - complementaryAnglePitch
    PtermPitch = PAnglePitch * ErrorAnglePitch
    ItermPitch = PrevItermAnglePitch + (IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * (t / 2))
    ItermPitch = 400 if ItermPitch > 400 else (-400 if ItermPitch < -400 else ItermPitch)
    DtermPitch = DAnglePitch * ((ErrorAnglePitch - PrevErrorAnglePitch) / t)
    PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch
    PIDOutputPitch = 400 if PIDOutputPitch > 400 else (-400 if PIDOutputPitch < -400 else PIDOutputPitch)
    DesiredRatePitch = PIDOutputPitch
    PrevErrorAnglePitch = ErrorAnglePitch
    PrevItermAnglePitch = ItermPitch

    # rate PID
    ErrorRateRoll = DesiredRateRoll - RateRoll
    ErrorRatePitch = DesiredRatePitch - RatePitch
    ErrorRateYaw = DesiredRateYaw - RateYaw

    PtermRoll = PRateRoll * ErrorRateRoll
    ItermRoll = PrevItermRateRoll + (IRateRoll * (ErrorRateRoll + PrevErrorRateRoll) * (t / 2))
    ItermRoll = 400 if ItermRoll > 400 else (-400 if ItermRoll < -400 else ItermRoll)
    DtermRoll = DRateRoll * ((ErrorRateRoll - PrevErrorRateRoll) / t)
    PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll
    PIDOutputRoll = 400 if PIDOutputRoll > 400 else (-400 if PIDOutputRoll < -400 else PIDOutputRoll)
    InputRoll = PIDOutputRoll
    PrevErrorRateRoll = ErrorRateRoll
    PrevItermRateRoll = ItermRoll

    PtermPitch = PRatePitch * ErrorRatePitch
    ItermPitch = PrevItermRatePitch + (IRatePitch * (ErrorRatePitch + PrevErrorRatePitch) * (t / 2))
    ItermPitch = 400 if ItermPitch > 400 else (-400 if ItermPitch < -400 else ItermPitch)
    DtermPitch = DRatePitch * ((ErrorRatePitch - PrevErrorRatePitch) / t)
    PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch
    PIDOutputPitch = 400 if PIDOutputPitch > 400 else (-400 if PIDOutputPitch < -400 else PIDOutputPitch)
    InputPitch = PIDOutputPitch
    PrevErrorRatePitch = ErrorRatePitch
    PrevItermRatePitch = ItermPitch

    PtermYaw = PRateYaw * ErrorRateYaw
    ItermYaw = PrevItermRateYaw + (IRateYaw * (ErrorRateYaw + PrevErrorRateYaw) * (t / 2))
    ItermYaw = 400 if ItermYaw > 400 else (-400 if ItermYaw < -400 else ItermYaw)
    DtermYaw = DRateYaw * ((ErrorRateYaw - PrevErrorRateYaw) / t)
    PIDOutputYaw = PtermYaw + ItermYaw + DtermYaw
    PIDOutputYaw = 400 if PIDOutputYaw > 400 else (-400 if PIDOutputYaw < -400 else PIDOutputYaw)
    InputYaw = PIDOutputYaw
    PrevErrorRateYaw = ErrorRateYaw
    PrevItermRateYaw = ItermYaw

    if InputThrottle > 1800:
        InputThrottle = 1800

    MotorInput1 = (InputThrottle - InputRoll - InputPitch - InputYaw)
    MotorInput2 = (InputThrottle - InputRoll + InputPitch + InputYaw)
    MotorInput3 = (InputThrottle + InputRoll + InputPitch - InputYaw)
    MotorInput4 = (InputThrottle + InputRoll - InputPitch + InputYaw)

    MotorInput1 = 1999 if MotorInput1 > 2000 else MotorInput1
    MotorInput2 = 1999 if MotorInput2 > 2000 else MotorInput2
    MotorInput3 = 1999 if MotorInput3 > 2000 else MotorInput3
    MotorInput4 = 1999 if MotorInput4 > 2000 else MotorInput4

    if MotorInput1 < ThrottleIdle: MotorInput1 = ThrottleIdle
    if MotorInput2 < ThrottleIdle: MotorInput2 = ThrottleIdle
    if MotorInput3 < ThrottleIdle: MotorInput3 = ThrottleIdle
    if MotorInput4 < ThrottleIdle: MotorInput4 = ThrottleIdle

    if ReceiverValue[2] < 1030:  # disarm
        MotorInput1 = ThrottleCutOff
        MotorInput2 = ThrottleCutOff
        MotorInput3 = ThrottleCutOff
        MotorInput4 = ThrottleCutOff

        PrevErrorRateRoll = 0; PrevErrorRatePitch = 0; PrevErrorRateYaw = 0
        PrevItermRateRoll = 0; PrevItermRatePitch = 0; PrevItermRateYaw = 0
        PrevErrorAngleRoll = 0; PrevErrorAnglePitch = 0
        PrevItermAngleRoll = 0; PrevItermAnglePitch = 0

    # motor write
    mot1.writeMicroseconds(int(MotorInput1))
    mot2.writeMicroseconds(int(MotorInput2))
    mot3.writeMicroseconds(int(MotorInput3))
    mot4.writeMicroseconds(int(MotorInput4))

    # timing guard to keep 4ms loop period
    while utime.ticks_diff(utime.ticks_us(), LoopTimer) < int(t * 1000000):
        pass
    LoopTimer = utime.ticks_us()

if __name__ == "__main__":
    setup()
    while True:
        loop()
