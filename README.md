# HeROS_micro_ros
micro-ROS app for ESP32 with FreeRTOS

### ESP-IDF installation
1. It's highly recommend to install the ESP-IDF as VSCode Extension using [this manual](https://github.com/espressif/vscode-esp-idf-extension/blob/master/docs/tutorial/install.md).
2. Choose **Express** setup mode.


sudo apt-get remove brltty

sudo dmesg --follow

dialout group 

mkdir uros_ws

### App:
```
mkdir app_ws
mkdir src
cd app_ws/src
git clone git@github.com:micro-ROS/micro_ros_espidf_component.git -b humble
cd micro_ros_espidf_component/
source ~/esp/v5.2.1/esp-idf/export.sh
pip3 install catkin_pkg lark-parser colcon-common-extensions
cd <app_dir>
idf.py set-target esp32
idf.py menuconfig
idf.py build flash monitor
```


### Agent:
```
mkdir agent_ws
mkdir src
cd agent_ws/src
git clone git@github.com:micro-ROS/micro_ros_setup.git -b humble
source /opt/ros/humble/setup.bash
colcon build
. install/setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
. install/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -v6
```
