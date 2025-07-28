# xtsdk\_ros

This code is confidential.

## This ROS package supports:

x86: ROS1 Melodic and ROS2 Foxy, Humble, Jazzy versions
aarch64: ROS1 Noetic and ROS2 Foxy, Humble, Jazzy versions
The chip exploration ROS functionality is implemented based on xtsdk\_cpp. Please refer to the readme.md document in the xtsdk\_cpp subdirectory for more details.

## ROS Version Selection

Copy xtsdk\_ros to the src directory of your workspace.

Navigate to the xtsdk\_ros directory.

Run the following commands:

## bash

chmod +x selros.sh  
./selros.sh 1   # to select ROS1 version  
./selros.sh 2   # to select ROS2 version  
Modifying IP Address
Please modify the default connection IP address before use.

\####ROS1

Before using, please modify the IP address in cfg/xtsdk\_ros1.cfg:



If using IP connection:

&nbsp;	gen.add("usb\_com", bool\_t, 0, "SET USB", False)

&nbsp;	gen.add("connect\_address", str\_t, 0, "connect address ip/serial", "192.168.2.101")

The IP address here is the LiDAR address.



If using USB connection:

&nbsp;	sudo chmod 777 /dev/ttyACM0

&nbsp;	gen.add("usb\_com", bool\_t, 1, "SET USB", True)



Then proceed with the compilation.



\####ROS2

&nbsp;	Before using, please modify the IP address in cfg/xtsdk\_ros2.yaml:



&nbsp;	If using IP connection:

&nbsp;	usb\_com: false

&nbsp;	connect\_address: 192.168.0.101

&nbsp;	The IP address here is the LiDAR address.



If using USB connection:

sudo chmod 777 /dev/ttyACM0

usb\_com: trueCompilation Environment
Supports Ubuntu 18.04 or Ubuntu 20.04
Supports ROS Melodic or ROS2 Foxy
Dependencies:
Boost: Configure the xtsdk CMakeLists.txt file according to your Boost installation path.
OpenCV: Install the OpenCV library using:
bash
sudo apt install libopencv-dev
----------------ROS1 -----------------------------------
Compilation
Load the ROS1 environment:

bash
source /opt/ros/melodic/setup.bash  
catkin\_make install
Running
bash
source install/setup.bash  
roslaunch xtsdk\_ros xtsdk\_ros1.launch
Parameter Tuning
During the initial parameter reading process, if the radar firmware is version 2.20 or above, it will automatically update device parameters to the tuning interface; otherwise, it will read from xtsdk\_ros/cfg/xintan.xtcfg.

----------------ROS2-----------------------------------
Compilation
Load the ROS2 environment:

bash
source /opt/ros/foxy/setup.bash
colcon build
Running
bash
source install/setup.bash
ros2 launch xtsdk\_ros xtsdk\_ros2.py
Parameter Tuning
Use rqt\_reconfigure for parameter tuning. For radar firmware version 2.20 or above, parameters will automatically update to the tuning interface; otherwise, it will read from xtsdk\_ros/cfg/xintan.xtcfg.

## NOTE

In ROS2, if the radar firmware is version 2.20 or above and you cannot update the GUI in real-time while using rqt\_reconfigure, please refer to:
https://github.com/ros-visualization/rqt\_reconfigure/commit/3752328747b6bae95db44f432c016df27362c88e

----------------Default Parameter Description------------------------
For both ROS1 and ROS2, the default configuration is read from the Windows host PC output configuration file xintan.xtcfg. If this configuration file or other parameters are missing, dynamic parameter files will be read (ROS1 uses xtsdk\_ros1.cfg, and ROS2 uses xtsdk\_ros2.yaml).

----------------Dynamic Parameter Description------------------------
image\_type (xtsdk output):
0: Depth Distance Image
1: Depth Image + Signal Amplitude Image
2: Grayscale Image
3: Point Cloud Output
4: Point Cloud + Signal Strength Output
camera/hdr\_mode: (HDR mode)
0: Off
1: On
integration\_time\_tof\_1: Integration time 1 value, unit: us, range: 0~2000
integration\_time\_tof\_2: Integration time 2 value, unit: us, range: 0~2000 (valid when HDR is on)
integration\_time\_tof\_3: Integration time 3 value, unit: us, range: 0~2000 (valid when HDR is on)
freq1: Frequency corresponding to integration time 1
freq2: Frequency corresponding to integration time 2
freq3: Frequency corresponding to integration time 3
integration\_time\_gray: Grayscale integration time value, unit: us, range: 0~20000 (for measuring ambient light)
min\_amplitude: Minimum effective signal strength value, point position LBS, range: 0~1000
frequency\_modulation: (TOF adjustment frequency)
0: 12MHz
1: 6MHz
----------------Topic Description------------------------
/xtsdk\_ros/xtsdk\_node/amplitude\_image\_raw: Signal amplitude, grayscale image Image Topic
/xtsdk\_ros/xtsdk\_node/distance\_image\_raw: Depth image Image Topic
/xtsdk\_ros/xtsdk\_node/gray\_image\_raw: Depth image Image Topic
/xtsdk\_ros/xtsdk\_node/points: Point cloud topic
-----------------PCD to BAG--------------------------
bash
./pcd\_to\_bag\_converter "your\_pcd\_path" "your\_bag\_file" "topic\_name"

