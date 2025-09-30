# MicroPython port that preserves original class name and method names.
# This replaces ESP32_Servo.cpp/.h with equivalent behavior using machine.PWM

from machine import Pin, PWM

# preserve original defines
DEFAULT_uS_LOW = 1000
DEFAULT_uS_HIGH = 2000

MIN_PULSE_WIDTH = 500
MAX_PULSE_WIDTH = 2500
DEFAULT_PULSE_WIDTH = 1500
REFRESH_CPS = 50  # 50Hz

class Servo:
    def __init__(self):
        # preserve original attributes as much as possible
        self.pinNumber = 0  # Changed from -1 to 0 to match C++ version
        self.min = DEFAULT_uS_LOW
        self.max = DEFAULT_uS_HIGH
        self.pwm = None
        self._last_us = DEFAULT_PULSE_WIDTH

    def attach(self, pin, min=DEFAULT_uS_LOW, max=DEFAULT_uS_HIGH):
        # recommended pins are ignored; MicroPython allows any valid GPIO
        self.pinNumber = pin
        self.min = min if min >= MIN_PULSE_WIDTH else MIN_PULSE_WIDTH
        self.max = max if max <= MAX_PULSE_WIDTH else MAX_PULSE_WIDTH

        p = Pin(self.pinNumber, Pin.OUT)
        self.pwm = PWM(p, freq=REFRESH_CPS)
        # initialize to default pulse width
        self.writeMicroseconds(DEFAULT_PULSE_WIDTH)
        # return a non-zero value like original attach (channel number ignored in MicroPython)
        return 1

    def setPeriodHertz(self, freq):
        if self.pwm:
            self.pwm.freq(freq)

    def detach(self):
        if self.pwm:
            self.pwm.deinit()
            self.pwm = None
        self.pinNumber = 0  # Changed from -1 to 0 to match C++ version

    def write(self, value):
        # preserve original semantics: <500 means degree
        if value < MIN_PULSE_WIDTH:
            if value < 0:
                value = 0
            elif value > 180:
                value = 180
            # map angle to microseconds
            us = int((self.max - self.min) * (value / 180.0) + self.min)
            self.writeMicroseconds(us)
        else:
            self.writeMicroseconds(value)

    def writeMicroseconds(self, value):
        if not self.pwm:
            return
        # clamp to min/max
        if value < self.min:
            value = self.min
        elif value > self.max:
            value = self.max

        self._last_us = value

        # Prefer duty_ns if available (exact high-time in ns)
        if hasattr(self.pwm, "duty_ns"):
            # value is microseconds high-time; convert to ns
            self.pwm.duty_ns(value * 1000)
            return

        # Fallback: duty_u16 (16-bit)
        period_us = int(1000000 // self.pwm.freq())  # period in us (50Hz -> 20000us)
        if hasattr(self.pwm, "duty_u16"):
            duty_u16 = int((value / period_us) * 65535)
            if duty_u16 < 0:
                duty_u16 = 0
            elif duty_u16 > 65535:
                duty_u16 = 65535
            self.pwm.duty_u16(duty_u16)
            return

        # Fallback: duty (0-1023 on ESP32)
        duty_max = 1023
        duty = int((value / period_us) * duty_max)
        if duty < 0:
            duty = 0
        elif duty > duty_max:
            duty = duty_max
        self.pwm.duty(duty)

    def readMicroseconds(self):
        return self._last_us

    def read(self):
        # return as angle 0..180 mapped from last_us
        us = self.readMicroseconds()
        if us <= self.min:
            return 0
        if us >= self.max:
            return 180
        return int((us - self.min) * 180.0 / (self.max - self.min))

    def attached(self):
        return self.pwm is not None
        
    # ESP32-specific functions (stubs for compatibility)
    def setTimerWidth(self, value):
        # MicroPython doesn't expose timer width control
        pass
        
    def readTimerWidth(self):
        # Return default timer width for compatibility
        return 16
