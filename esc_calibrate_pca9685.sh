#!/bin/bash

python3 << 'EOF'
import smbus
import time

I2C_ADDR = 0x40
ESC_CH = 0

MODE1 = 0x00
PRESCALE = 0xFE
LED0_ON_L = 0x06

bus = smbus.SMBus(1)

def write(reg, val):
    bus.write_byte_data(I2C_ADDR, reg, val)

def set_pwm_freq(freq):
    prescale = int(25000000 / (4096 * freq) - 1)
    oldmode = bus.read_byte_data(I2C_ADDR, MODE1)
    write(MODE1, (oldmode & 0x7F) | 0x10)
    write(PRESCALE, prescale)
    write(MODE1, oldmode)
    time.sleep(0.005)
    write(MODE1, oldmode | 0x80)

def set_pwm_us(channel, us):
    ticks = int(us * 4096 / 20000)
    base = LED0_ON_L + 4 * channel
    write(base, 0)
    write(base + 1, 0)
    write(base + 2, ticks & 0xFF)
    write(base + 3, ticks >> 8)
    print(f"[INFO] PWM = {us} µs")

print("=== ESC CALIBRATION START ===")

set_pwm_freq(50)

set_pwm_us(ESC_CH, 2000)
time.sleep(5)

set_pwm_us(ESC_CH, 1000)
time.sleep(2)

set_pwm_us(ESC_CH, 1200)
time.sleep(2)

set_pwm_us(ESC_CH, 1300)
time.sleep(2)

set_pwm_us(ESC_CH, 1600)
time.sleep(1)

set_pwm_us(ESC_CH, 0)
print("ESC stopped")
EOF
