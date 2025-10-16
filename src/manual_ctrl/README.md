# READ ME

## Nodes Description

### 1. `joy_raw`
- Reads raw joystick data from `/joy` device.
- Publishes a ramped, processed joystick message on `/drift_pilot/joy_ramped`.
- Ensures smooth control input translation for further processing.

### 2. `joy_drive`
- Subscribes to `/drift_pilot/joy_ramped`.
- Converts joystick axes into velocity and steering commands.
- Limits maximum speed and steering angle for safety.
- Publishes `AckermannDriveStamped` commands to steer and move the vehicle.

### 3. `motor_signals`
- Subscribes to `AckermannDriveStamped` commands.
- Converts drive commands into PWM signals using `pigpio`.
- Controls GPIO pins for the ESC (speed controller) and the steering servo.
- Handles initialization and safe shutdown with neutral signals.
run the following command to start the pigpiod daemon and the ros2 joystick reades:

## To launch 
```bash
sudo pigpiod
ros2 run joy_linux joy_node --ros-args -r joy:=driftpilot_joy/joy_ramped
```
then run the launch files

### 1. manual_ctrl
```bash
ros2 launch Manual_ctrl all_manual_ctrl.launch.py
```

### 2. joy_raw
```bash
ros2 launch Manual_ctrl joy_raw.launch.py
```

### 3.joy_drive
```bash
ros2 launch Manual_ctrl joy_drive.launch.py
```

### 4. motor_signals
```bash
ros2 launch Manual_ctrl motor_signals.launch.py
```
