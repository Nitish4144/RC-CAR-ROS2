# READ ME

run the following command to start the pigpiod daemon and the ros2 joystick reades:

```bash
sudo pigpiod
ros2 run joy_linux joy_node --ros-args -r joy:=driftpilot_joy/joy_ramped
```
then run the launch files
