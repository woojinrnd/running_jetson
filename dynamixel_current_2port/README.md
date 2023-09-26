# VSCODE
ctrl+shift+p --> Project Tree  
ctrl+shift+v --> README.md   


# DEPENDENCIES
xsens mti driver  
dynamixel_sdk  
realsense_sdk

# INSTALL
## xsens mti driver  
#2-1. MT 소프트웨어 다운로드  
https://www.xsens.com/setup  

#2-2. 소프트웨어 설치  
좌측의 Xsens MTi  클릭 → 자동으로 MT_Software_Suite_linux_x64_2022.0.2 다운로드  
cd Downloads && sudo ./mtsdk_linux-x64_2022.0.2.sh  
sudo apt-get install sharutils  
default 경로 설정 : /usr/local/xsens  
cd /usr/local/xsens  

#2-3. catkin_ws 빌드 작업  
```bash

mkdir -p xsense/src
cd /usr/local/xsens  
sudo cp -r xsens_ros_mti_driver ~/xsense/src  
cd ~/xsense/src  
sudo chmod 777 -R xsens_ros_mti_driver  
cd  
pushd ~/xsense/src/xsens_ros_mti_driver/lib/xspublic && make && popd  
catkin_make    
source devel/setup.bash  
roslaunch xsens_mti_driver display.launch  

```

#2-4. Permission Denied Error 해결  
```bash
ls -l /dev/ttyUSB0  
id  
sudo usermod -a -G dialout [username]  
* 컴퓨터를 재부팅해야 적용됨  
```

#3-2. ROS 노드 실행  
```bash  

roslaunch xsens_mti_driver xsens_mti_node.launch  
rostopic echo /imu/data  

```  

## dynamixel_sdk
* [dynamixel_sdk](http://wiki.ros.org/dynamixel_sdk)  
```bash  

sudo apt-get install ros-noetic-dynamixel-sdk

```

# BUILD  
```bash

mkdir -p [what you want folder name]/[src]  
cd [what you want folder name]/[src]  
git clone https://github.com/woojinrnd/dynamixel_current_2port.git  
cd ..  
catkin_make  

```  

# RUN
```bash  

source devel/setup.bash  
split 4 terminal  
<!-- [terminal 1] roscore   --> (if no roscore)  
[terminal 1] roslaunch xsens_mti_driver xsens_mti_node.launch  
[terminal 2] rosrun dynamixel_current_2port Sensor_node  
[terminal 3] rosrun dynamixel_current_2port Move_decision_node  
[terminal 4] rosrun dynamixel_current_2port dynamixel_current_2port  

```  

# ROS GRPAH
![0901(2)_rosgraph](https://github.com/woojinrnd/dynamixel_current_2port/assets/122770475/e200799a-3900-4e65-bb6a-d9b0a7ae3a8f)

# NODE
## Sensor_node  (400hz)  
```

- IMUcallbackThread (400hz) - Subscribe [imu/data] from xsens_mti_node  
- SensorPublishThread (200hz) - Publish  
[Angle/x, Angle/y, Angle/z]    
[filtered/Velocity_Complementary/x, filtered/Velocity_Complementary/y, filtered/Velocity_Complementary/z]  

```

## Move_decision_node  (100hz)
```

- processThread (100hz)  - Determine the mode flag from img_proc    
- webcam_thread (30hz)  -  2D WebCam img_proc  
- realsense_thread (30hz)  -  Depth Camera img_proc  
- callbackThread (100hz)  -  Subscribe [Angle/y] from Sensor_node  
Server : SendMotion.srv (Select_motion / Turn_angle / UD_neckangle / RL_neckangle / Emergency )  

```

## dynamixel_current_2port (100hz)
```

- callbackThread (100hz)  -  Subscribe [FSR/L, FSR/R] from Arduino nano   
Client : SendMotion.srv (finish / finish / finish / finish / finish / finish )    
- IMUThread (200hz) - Subscribe [filtered/Velocity_Complementary/x, filtered/Velocity_Complementary/y, filtered/Velocity_Complementary/z] from Sensor_node  

```

## xsens_mti_node (400hz)
```

- Publish /imu/data (400hz)  

```

# Souce Tree
```
src
├── 0515_rosgraph.png
├── 0607_rosgraph2.png
├── 0607_rosgraph.png
├── 0621_rosgraph.png
├── 0721_rosgraph.png
├── 0730(2)_rosgraph.png
├── 0730_rosgraph.png
├── 0901(2)_rosgraph.png
├── 0901_rosgraph.png
├── CMakeLists.txt -> /opt/ros/noetic/share/catkin/cmake/toplevel.cmake
├── dynamixel_current_2port
│   ├── CMakeLists.txt
│   ├── config
│   │   └── parameters.yml
│   ├── include
│   │   ├── callback.hpp
│   │   ├── dynamixel_controller.hpp
│   │   ├── dynamixel.hpp
│   │   ├── img_proc.hpp
│   │   ├── Move_decision.hpp
│   │   ├── sensor.hpp
│   │   └── Walkingpattern_generator.hpp
│   ├── launch
│   │   ├── dynamixel_current_2port.launch
│   │   └── Move_decision.launch
│   ├── package.xml
│   ├── src
│   │   ├── callback.cpp
│   │   ├── dynamixel_controller.cpp
│   │   ├── dynamixel.cpp
│   │   ├── fsr_ux_420_short.ino
│   │   ├── img_proc.cpp
│   │   ├── main_2.cpp
│   │   ├── main.cpp
│   │   ├── Move_decision.cpp
│   │   ├── Move_decision_node.cpp
│   │   ├── sensor.cpp
│   │   ├── Sensor_node.cpp
│   │   └── Walkingpattern_generator.cc
│   └── srv
│       ├── Emergency.srv
│       ├── RL_NeckAngle.srv
│       ├── Select_Motion.srv
│       ├── SendMotion.srv
│       ├── Turn_Angle.srv
│       └── UD_NeckAngle.srv
└── README.md
```
