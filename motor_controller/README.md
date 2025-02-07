
# How to use 


* git clone this repo on both dev machine and pi
``` 
git clone https://github.com/Rammani28/motor_control_ws.git 
```
* On dev machine, run the following
``` 
cd ~/ros_ws
colcon build --symlink-install
source install/setup.bash
ros2 run motor_controller led_control_jj
ros2 run joy joy_node # on new terminal
```
* run publisher node on dev machine

* run joy node on dev machine on new terminal

* run listener node on pi

* On pi, run the following
```
cd ~/ros_ws
colcon build --symlink-install
source install/setup.bash
ros2 run motor_controller led_subscriber_jj
```
### Note:
Pressing X button of joystick turns the led On, while default option is off. gpio pin used is 23 or physical pin 16.
