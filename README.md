# HeROS_micro_ros

### Description
Controlling environmental obstacles as part of the HeROS project.


### Hardware:
* [FireBeetle ESP32 IoT Microcontroller](https://www.dfrobot.com/product-1590.html)


### ESP-IDF installation
1. It's highly recommend to install the ESP-IDF as VSCode Extension using [this manual](https://github.com/espressif/vscode-esp-idf-extension/blob/master/docs/tutorial/install.md).
2. Choose **Express** setup mode.

### Potential issues when connecting to ESP32 over USB
1. Add yourself to `dialout` group.
2. `sudo apt-get remove brltty`

### Workspace organization
```
~/uros_ws |- app_ws
          |- agent_ws

```

### micro-ROS application:
```
cd uros_ws/app_ws/src/
git clone git@github.com:jkaniuka/HeROS_micro_ros.git
cd HeROS_micro_ros/
source ~/esp/v5.2.1/esp-idf/export.sh
pip3 install catkin_pkg lark-parser colcon-common-extensions
cd obstacles_controller/
idf.py set-target esp32
idf.py menuconfig
idf.py build flash monitor
```

### micro-ROS agent:
```
cd uros_ws/agent_ws/src/
git clone git@github.com:micro-ROS/micro_ros_setup.git -b humble
source /opt/ros/humble/setup.bash
colcon build
. install/setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
. install/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
```

## Example command to rotate an obstacle
```bash
ros2 service call /obstacle_1 example_interfaces/srv/SetBool "{data: True}"
```
