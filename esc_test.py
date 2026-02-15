from gpiozero import Servo
from time import sleep

esc = Servo(
    18,
    min_pulse_width=0.001,   # 1000 us
    max_pulse_width=0.002   # 2000 us
)

def set_val(v, t=2):
    esc.value = v
    print(f"ESC value = {v}")
    sleep(t)

print("=== ESC DEADBAND FINDER ===")

print("Arming at STOP (0.0)")
sleep(2)
set_val(0.0, 4)

print("Slow forward ramp")
v = 0.0
while v <= 0.4:
    set_val(v, 1.5)
    v += 0.05

print("Back to STOP")
set_val(0.0, 3)

print("Slow reverse ramp")
v = 0.0
while v >= -0.4:
    set_val(v, 1.5)
    v -= 0.05

print("Stopping")
set_val(0.0, 3)

esc.detach()
print("=== DONE ===")
