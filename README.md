# ESP32 飞控（MicroPython 版本）

本仓库在 `src/` 目录下提供了将原始 Arduino/C++ 工程（`Anglemode_flightcontroller_ver3.1.ino` 与 `ESP32_Servo.*`）完整迁移到 MicroPython 的实现。迁移版本严格保留了所有原始的函数名与变量名，以便于对照原始 Arduino 项目进行调试与二次开发。

本 README 面向技术人员，介绍硬件连接、运行部署方法、控制逻辑（IMU 读取、滤波、PID、混控、解锁/安全）以及常见问题与二次开发建议。

---

## 文件结构

- `src/Anglemode_flightcontroller_ver3.1.py`
  - 主控制循环与所有飞控逻辑（setup/loop、IMU 数据采集、互补滤波、双层 PID、混控、解锁/安全保护）。
  - 严格保留原始函数与变量命名：如 `setup()`, `loop()`, `gyro_signals()`, `kalman_1d()`, `PID` 相关变量、接收机/电机输入输出变量等。

- `src/ESP32_Servo.py`
  - 替代 `ESP32_Servo.cpp/.h` 的 MicroPython 版本，提供相同的类与方法接口：
    - `Servo.attach(pin, min, max)`
    - `Servo.writeMicroseconds(us)`
    - `Servo.readMicroseconds()`
    - `Servo.setPeriodHertz(freq)`
    - `Servo.detach()`
    - `Servo.attached()`
    - `Servo.setTimerWidth(width)`, `Servo.readTimerWidth()`（兼容桩函数）

- 原始 Arduino/C++ 文件保留用于对照：
  - `src/Anglemode_flightcontroller_ver3.1.ino`
  - `src/ESP32_Servo.cpp`
  - `src/ESP32_Servo.h`

---

## 硬件要求

- 主控：ESP32-S3（其他 ESP32 变体通常也可，但需确认引脚与 PWM/定时器可用性）
- 传感器：MPU6050（I2C）
- 无线接收机（PWM输出，6通道）
- 电子调速器（ESC）+ 无刷电机（四电机）
- 电源：确保稳定供电，尤其是 ESC 与电机的供电链路
- 安全措施：测试前请拆除螺旋桨或采用牢固固定装置

---

## 引脚映射

- 电机（PWM输出，使用 `ESP32_Servo`）
  - `mot1_pin = 13`
  - `mot2_pin = 12`
  - `mot3_pin = 14`
  - `mot4_pin = 27`

- 接收机通道（PWM输入，中断捕获脉宽，内部上拉）
  - `channel_1_pin = 34`（Roll）
  - `channel_2_pin = 35`（Pitch）
  - `channel_3_pin = 32`（Throttle）
  - `channel_4_pin = 33`（Yaw）
  - `channel_5_pin = 25`
  - `channel_6_pin = 26`

- I2C（MPU6050）
  - `SDA = 21`
  - `SCL = 22`
  - 速率：`400kHz`

- LED 指示
  - `Pin(15, OUT)`（启动时 9 次闪烁流程）

注意：ESP32 的 34/35 为输入专用引脚，适用于捕获接收机 PWM 脉宽。

---

## 软件环境准备

- 在 ESP32-S3 上烧录 MicroPython 固件（官方或社区版本）。
- 通过 Thonny/mpremote/rshell 等工具将以下文件复制到设备文件系统：
  - `src/ESP32_Servo.py`
  - `src/Anglemode_flightcontroller_ver3.1.py`

> 如果需要上电自动运行，建议将 `Anglemode_flightcontroller_ver3.1.py` 重命名为 `main.py` 或在 `main.py` 中导入并启动相应逻辑。

---

## 部署与运行

1. 将硬件按“引脚映射”一节正确连接。
2. 上电前确保 ESC 与电机的供电与信号地正确共地。
3. 将 `ESP32_Servo.py` 与 `Anglemode_flightcontroller_ver3.1.py` 复制到板子。
4. 在 REPL 或 `main.py` 中启动：
   - 方式 A（自动运行）：将 `Anglemode_flightcontroller_ver3.1.py` 重命名为 `main.py`，上电自动执行
   - 方式 B（手动运行）：在 REPL 中执行：
     - `import Anglemode_flightcontroller_ver3_1`
     - `Anglemode_flightcontroller_ver3_1.setup()`
     - 然后循环调用 `Anglemode_flightcontroller_ver3_1.loop()`（或如文件底部 `__main__` 中的结构）

> 当前 `Anglemode_flightcontroller_ver3.1.py` 文件内部带有：
> ```
> if __name__ == "__main__":
>     setup()
>     while True:
>         loop()
> ```
> 如果直接作为主脚本运行（例如改名为 `main.py` 或通过 Thonny 直接运行该文件），会自动执行。

---

## 控制逻辑概览

- 循环周期：`t = 0.004`（4ms），通过 `LoopTimer` 与 `utime.ticks_us()` 保持节拍。
- IMU 读取：
  - 每次循环写入寄存器配置以匹配 C++ 实现：
    - `0x1A = 0x05`（DLPF）
    - `0x1C = 0x10`（加速度量程）
    - `0x1B = 0x08`（陀螺仪量程）
  - 加速度值转换：`Acc* = Raw / 4096.0`
  - 陀螺仪角速度：`Rate* = Raw / 65.5`
  - 角度计算：基于 `atan` 与欧拉投影（度数 57.29）

- 传感器校准（固定常量，源自原始 C++）：
  - `RateCalibrationRoll = 0.27`
  - `RateCalibrationPitch = -0.85`
  - `RateCalibrationYaw = -2.09`
  - `AccXCalibration = 0.03`
  - `AccYCalibration = 0.01`
  - `AccZCalibration = -0.07`

- 互补滤波：
  - `complementaryAngle* = 0.991 * (prev + Rate* * t) + 0.009 * Angle*`
  - 输出限幅：`±20°`

- 接收机输入映射：
  - `DesiredAngleRoll = 0.1 * (channel_1 - 1500)`
  - `DesiredAnglePitch = 0.1 * (channel_2 - 1500)`
  - `InputThrottle = channel_3`
  - `DesiredRateYaw = 0.15 * (channel_4 - 1500)`
  - 接收机脉宽通过 GPIO 中断在上升/下降沿测量 `us` 脉宽并存入 `ReceiverValue[]`

- 双层 PID 控制：
  1. 角度环（Angle → Rate Demand）
     - `PAngleRoll = 2; IAngleRoll = 0.5; DAngleRoll = 0.007`
     - Pitch 使用同样参数
     - 输出限幅 `±400`，积分限幅 `±400`
  2. 角速度环（Rate → Motor Mix Input）
     - Roll：`PRateRoll = 0.625; IRateRoll = 2.1; DRateRoll = 0.0088`
     - Pitch 使用同样参数；Yaw：`PRateYaw = 4; IRateYaw = 3; DRateYaw = 0`
     - 同样限幅与积分限幅

- 电机混控：
  - `MotorInput1 = (Throttle - Roll - Pitch - Yaw)`
  - `MotorInput2 = (Throttle - Roll + Pitch + Yaw)`
  - `MotorInput3 = (Throttle + Roll + Pitch - Yaw)`
  - `MotorInput4 = (Throttle + Roll - Pitch + Yaw)`
  - 上限裁剪：>2000 → 1999
  - 下限保护：<`ThrottleIdle(1170)` → `ThrottleIdle`
  - 失效保护/解锁逻辑：
    - 若 `ReceiverValue[2] < 1030`（低油门），则所有电机输出 `ThrottleCutOff(1000)`
    - 并清零所有 PID 误差与积分项（确保重新解锁时无旧积分残留）

---

## ESC 与 PWM

- ESC 频率：`ESCfreq = 500Hz`
- 通过 `ESP32_Servo` 类以 `writeMicroseconds(us)` 输出脉宽（1000~2000us）。
- `attach(pin, min, max)` 后需调用 `setPeriodHertz(ESCfreq)` 设置 PWM 频率。
- 初始化顺序（与原 C++ 保持一致）：
  - 逐个 `attach` → 间隔延时 → `setPeriodHertz` → 延时 → 初始写入 `1000us`（安全低油门）

---

## LED 指示（启动序列）

- 启动时进行 9 次闪烁（完全匹配原始 C++ 的时序），便于技术人员快速确认上电流程已完成。

---

## 性能与实时性注意

- 循环周期固定为 4ms，使用忙等待守时：
  - `while utime.ticks_diff(utime.ticks_us(), LoopTimer) < int(t * 1000000):`
- 若需调整循环周期，需同步修改 `t`、`ESCfreq` 与所有与时间相关的参数（如 PID 系数）。

- 接收机脉宽测量依赖 GPIO 中断（上升/下降沿），在高干扰环境可能出现毛刺，建议：
- 硬件端加滤波或合理布线
- 软件端可增加脉宽范围校验（如 900~2100us）以剔除异常

---

## 常见问题

- 电机不转或异常抖动：
- 检查 ESC 是否已正确校准，确保初始写入 `1000us` 后再逐步加油门
- 检查供电与地线连接
- 检查 `ESCfreq` 与具体 ESC 的兼容性（500Hz 对大多数兼容）

- 姿态狂跳/不稳定：
- 检查 MPU6050 连接与供电
- 确保 I2C 线缆短且屏蔽良好，避免干扰
- 检查机体固定，确保 IMU 无松动
- 适当调整 PID（先从 Rate 环开始，小步调整）

- 接收机无输入：
- 确认接收机输出为 3.3V 逻辑电平的 PWM
- 确认引脚映射正确（34/35/32/33/25/26）
- 检查上拉与中断是否正常（已设置 `Pin.PULL_UP` 与双沿触发）

---

## 二次开发建议

- 变量与函数名与原始 C++ 完全一致，建议对照 `src/Anglemode_flightcontroller_ver3.1.ino` 与 `src/Anglemode_flightcontroller_ver3.1.py`进行逐段比对。
- 若需增加日志输出或上位机通信，建议在 `loop()` 尾部添加非阻塞式发送，并注意 4ms 时间预算。
- 若需切换滤波器（如改用 Kalman），可复用已有的 `kalman_1d()` 框架，保持 `Kalman1DOutput` 的数据接口一致。
- 若要变更引脚或频率，集中修改以下位置：
- 引脚：`mot*_pin`、`channel_*_pin`、`I2C(scl/sda)`
- PWM 频率：`ESCfreq`
- 循环周期：`t`（并同步检查所有与时间相关的参数）

---

## 安全声明

- 强烈建议在无桨测试或固定平台下进行初次调试。
- 始终保持低油门上电与解锁流程，避免误动作。
- 调试时建议在稳定桌面/支架上进行，避免飞控算法未调优造成风险。

---

## 许可证

请参阅仓库根目录的 `LICENSE` 文件。

---
