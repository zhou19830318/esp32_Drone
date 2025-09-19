from machine import Pin, PWM, I2C
import time, math
# ============ 全局参数 ============
RatePitch = 0.0
RateRoll = 0.0
RateYaw = 0.0
RateCalibrationRoll = 0.27
RateCalibrationPitch = -0.85
RateCalibrationYaw = -2.09
AccXCalibration = 0.03
AccYCalibration = 0.01
AccZCalibration = -0.07
ESCfreq = 500
t = 0.004 # 控制周期 4ms (250Hz)
LoopTimer = 0
# PID参数 (角度环)
PAngleRoll = 2
IAngleRoll = 0.5
DAngleRoll = 0.007
PAnglePitch, IAnglePitch, DAnglePitch = PAngleRoll, IAngleRoll, DAngleRoll
# PID参数 (角速度环)
PRateRoll = 0.625
IRateRoll = 2.1
DRateRoll = 0.0088
PRatePitch, IRatePitch, DRatePitch = PRateRoll, IRateRoll, DRateRoll
PRateYaw, IRateYaw, DRateYaw = 4, 3, 0
# Motor PWM
mot1 = PWM(Pin(13), freq=ESCfreq, duty=0)
mot2 = PWM(Pin(12), freq=ESCfreq, duty=0)
mot3 = PWM(Pin(14), freq=ESCfreq, duty=0)
mot4 = PWM(Pin(27), freq=ESCfreq, duty=0)
# Receiver channels
channel_pins = [34,35,32,33,25,26]
ReceiverValue = [1500,1500,1000,1500,1500,1500]
last_ticks = [0]*6
# Failsafe
ThrottleIdle = 1170
ThrottleCutOff = 1000
# Attitude
AccX=AccY=AccZ=0
AngleRoll=AnglePitch=0
complementaryAngleRoll=0
complementaryAnglePitch=0
# PID state vars
PrevErrorAngleRoll=PrevErrorAnglePitch=0
PrevItermAngleRoll=PrevItermAnglePitch=0
PrevErrorRateRoll=PrevErrorRatePitch=PrevErrorRateYaw=0
PrevItermRateRoll=PrevItermRatePitch=PrevItermRateYaw=0
InputRoll=InputPitch=InputYaw=0
DesiredRateRoll=DesiredRatePitch=DesiredRateYaw=0
# Motor outputs
MotorInput1=MotorInput2=MotorInput3=MotorInput4=0
# I2C MPU6050
i2c = I2C(0, scl=Pin(22), sda=Pin(21), freq=400000)
# ============ 工具函数 ============
def micros():
    return time.ticks_us()
def servo_write_us(pwm, us):
    period_us = 1_000_000 // ESCfreq
    duty = int((us / period_us) * 1023)
    pwm.duty(duty)
def channel_irq(pin):
    idx = channel_pins.index(pin.id())
    now = micros()
    global last_ticks, ReceiverValue
    if pin.value():
        last_ticks[idx] = now
    else:
        ReceiverValue[idx] = time.ticks_diff(now, last_ticks[idx])
# 绑定中断
for pnum in channel_pins:
    Pin(pnum, Pin.IN).irq(trigger=Pin.IRQ_RISING|Pin.IRQ_FALLING, handler=channel_irq)
# ============ MPU6050 读取 ============
def gyro_signals():
    global RateRoll,RatePitch,RateYaw,AccX,AccY,AccZ,AngleRoll,AnglePitch
    i2c.writeto_mem(0x68,0x1A,b'\x05')
    i2c.writeto_mem(0x68,0x1C,b'\x10')
    data_acc = i2c.readfrom_mem(0x68,0x3B,6)
    ax = int.from_bytes(data_acc[0:2],'big',signed=True)
    ay = int.from_bytes(data_acc[2:4],'big',signed=True)
    az = int.from_bytes(data_acc[4:6],'big',signed=True)
    i2c.writeto_mem(0x68,0x1B,b'\x08')
    data_gyro = i2c.readfrom_mem(0x68,0x43,6)
    gx = int.from_bytes(data_gyro[0:2],'big',signed=True)
    gy = int.from_bytes(data_gyro[2:4],'big',signed=True)
    gz = int.from_bytes(data_gyro[4:6],'big',signed=True)
    RateRoll = gx/65.5
    RatePitch = gy/65.5
    RateYaw = gz/65.5
    AccX = ax/4096
    AccY = ay/4096
    AccZ = az/4096
    RateRoll -= RateCalibrationRoll
    RatePitch -= RateCalibrationPitch
    RateYaw -= RateCalibrationYaw
    AccX -= AccXCalibration
    AccY -= AccYCalibration
    AccZ -= AccZCalibration
    AngleRoll = math.degrees(math.atan(AccY/math.sqrt(AccX*AccX+AccZ*AccZ)))
    AnglePitch = -math.degrees(math.atan(AccX/math.sqrt(AccY*AccY+AccZ*AccZ)))
# ============ PID方程 ============
def pid_equation(Error,P,I,D,PrevError,PrevIterm):
    Pterm = P*Error
    Iterm = PrevIterm + I*(Error+PrevError)*(t/2)
    Iterm = max(min(Iterm,400),-400)
    Dterm = D*((Error-PrevError)/t)
    PIDOutput = Pterm+Iterm+Dterm
    PIDOutput = max(min(PIDOutput,400),-400)
    return PIDOutput, Error, Iterm
# ============ 主循环 ============
def loop():
    global complementaryAngleRoll,complementaryAnglePitch
    global PrevErrorAngleRoll,PrevErrorAnglePitch
    global PrevItermAngleRoll,PrevItermAnglePitch
    global PrevErrorRateRoll,PrevErrorRatePitch,PrevErrorRateYaw
    global PrevItermRateRoll,PrevItermRatePitch,PrevItermRateYaw
    global DesiredRateRoll,DesiredRatePitch,DesiredRateYaw
    global InputRoll,InputPitch,InputYaw
    global MotorInput1,MotorInput2,MotorInput3,MotorInput4
    global LoopTimer
    while True:
        start = micros()
        # IMU
        gyro_signals()
        # Complementary filter
        complementaryAngleRoll = 0.991*(complementaryAngleRoll+RateRoll*t)+0.009*AngleRoll
        complementaryAnglePitch = 0.991*(complementaryAnglePitch+RatePitch*t)+0.009*AnglePitch
        complementaryAngleRoll = min(max(complementaryAngleRoll,-20),20)
        complementaryAnglePitch = min(max(complementaryAnglePitch,-20),20)
        # 输入目标
        DesiredAngleRoll = 0.1*(ReceiverValue[0]-1500)
        DesiredAnglePitch = 0.1*(ReceiverValue[1]-1500)
        InputThrottle = ReceiverValue[2]
        DesiredRateYaw = 0.15*(ReceiverValue[3]-1500)
        # ---- 外环 PID (角度环) ----
        ErrorAngleRoll = DesiredAngleRoll - complementaryAngleRoll
        DesiredRateRoll, PrevErrorAngleRoll, PrevItermAngleRoll = pid_equation(
            ErrorAngleRoll,PAngleRoll,IAngleRoll,DAngleRoll,PrevErrorAngleRoll,PrevItermAngleRoll
        )
        ErrorAnglePitch = DesiredAnglePitch - complementaryAnglePitch
        DesiredRatePitch, PrevErrorAnglePitch, PrevItermAnglePitch = pid_equation(
            ErrorAnglePitch,PAnglePitch,IAnglePitch,DAnglePitch,PrevErrorAnglePitch,PrevItermAnglePitch
        )
        # ---- 内环 PID (角速度环) ----
        ErrorRateRoll = DesiredRateRoll - RateRoll
        InputRoll, PrevErrorRateRoll, PrevItermRateRoll = pid_equation(
            ErrorRateRoll,PRateRoll,IRateRoll,DRateRoll,PrevErrorRateRoll,PrevItermRateRoll
        )
        ErrorRatePitch = DesiredRatePitch - RatePitch
        InputPitch, PrevErrorRatePitch, PrevItermRatePitch = pid_equation(
            ErrorRatePitch,PRatePitch,IRatePitch,DRatePitch,PrevErrorRatePitch,PrevItermRatePitch
        )
        ErrorRateYaw = DesiredRateYaw - RateYaw
        InputYaw, PrevErrorRateYaw, PrevItermRateYaw = pid_equation(
            ErrorRateYaw,PRateYaw,IRateYaw,DRateYaw,PrevErrorRateYaw,PrevItermRateYaw
        )
        # ---- 混控 ----
        if InputThrottle>1800: InputThrottle=1800
        MotorInput1 = InputThrottle - InputRoll - InputPitch - InputYaw
        MotorInput2 = InputThrottle - InputRoll + InputPitch + InputYaw
        MotorInput3 = InputThrottle + InputRoll + InputPitch - InputYaw
        MotorInput4 = InputThrottle + InputRoll - InputPitch + InputYaw
        # 限幅
        if MotorInput1 > 2000: MotorInput1 = 1999
        if MotorInput2 > 2000: MotorInput2 = 1999
        if MotorInput3 > 2000: MotorInput3 = 1999
        if MotorInput4 > 2000: MotorInput4 = 1999
        if MotorInput1 < ThrottleIdle: MotorInput1 = ThrottleIdle
        if MotorInput2 < ThrottleIdle: MotorInput2 = ThrottleIdle
        if MotorInput3 < ThrottleIdle: MotorInput3 = ThrottleIdle
        if MotorInput4 < ThrottleIdle: MotorInput4 = ThrottleIdle
        # 失控保护
        if ReceiverValue[2]<1030:
            MotorInput1=MotorInput2=MotorInput3=MotorInput4=ThrottleCutOff
            PrevErrorRateRoll=PrevErrorRatePitch=PrevErrorRateYaw=0
            PrevItermRateRoll=PrevItermRatePitch=PrevItermRateYaw=0
            PrevErrorAngleRoll=PrevErrorAnglePitch=0
            PrevItermAngleRoll=PrevItermAnglePitch=0
        # ---- 输出 ----
        servo_write_us(mot1,MotorInput1)
        servo_write_us(mot2,MotorInput2)
        servo_write_us(mot3,MotorInput3)
        servo_write_us(mot4,MotorInput4)
        # ---- 串口调试输出 ----
        print("RX:", ReceiverValue,
              "Angles:", round(AngleRoll,2), round(AnglePitch,2),
              "Comp:", round(complementaryAngleRoll,2), round(complementaryAnglePitch,2),
              "PID:", round(InputRoll,2), round(InputPitch,2), round(InputYaw,2),
              "Motors:", int(MotorInput1), int(MotorInput2), int(MotorInput3), int(MotorInput4))
        # ---- 定时循环 ----
        while time.ticks_diff(micros(),start) < int(t*1_000_000):
            pass
# ============ 初始化 ============
def setup():
    i2c.writeto_mem(0x68,0x6B,b'\x00') # 唤醒MPU
    servo_write_us(mot1,1000)
    servo_write_us(mot2,1000)
    servo_write_us(mot3,1000)
    servo_write_us(mot4,1000)
    time.sleep_ms(500)
    print("MPU6050 ready.")
setup()
loop()
