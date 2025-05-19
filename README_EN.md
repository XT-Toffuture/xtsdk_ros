# xtsdk_ros
This code is confidential.

## This ROS package supports:

x86: ROS1 Melodic and ROS2 Foxy, Humble, Jazzy versions
aarch64: ROS1 Noetic and ROS2 Foxy, Humble, Jazzy versions
The chip exploration ROS functionality is implemented based on xtsdk_cpp. Please refer to the readme.md document in the xtsdk_cpp subdirectory for more details.

## ROS Version Selection
Copy xtsdk_ros to the src directory of your workspace.

Navigate to the xtsdk_ros directory.

Run the following commands:

## bash
chmod +x selros.sh  
./selros.sh 1   # to select ROS1 version  
./selros.sh 2   # to select ROS2 version  
Modifying IP Address
Please modify the default connection IP address before use.

ROS1
Modify the IP address in the configuration file cfg/xtsdk_ros1.cfg:

plaintext
gen.add("connect_address", str_t, 0, "connect address ip/serial", "192.168.2.101")
ROS2
Modify the IP address in the configuration file cfg/xtsdk_ros2.yaml:

plaintext
connect_address: 192.168.2.101
Compilation Environment
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
catkin_make install
Running
bash
source install/setup.bash  
roslaunch xtsdk_ros xtsdk_ros1.launch
Parameter Tuning
During the initial parameter reading process, if the radar firmware is version 2.20 or above, it will automatically update device parameters to the tuning interface; otherwise, it will read from xtsdk_ros/cfg/xintan.xtcfg.

----------------ROS2-----------------------------------
Compilation
Load the ROS2 environment:

bash
source /opt/ros/foxy/setup.bash
colcon build
Running
bash
source install/setup.bash
ros2 launch xtsdk_ros xtsdk_ros2.py
Parameter Tuning
Use rqt_reconfigure for parameter tuning. For radar firmware version 2.20 or above, parameters will automatically update to the tuning interface; otherwise, it will read from xtsdk_ros/cfg/xintan.xtcfg.

## NOTE
In ROS2, if the radar firmware is version 2.20 or above and you cannot update the GUI in real-time while using rqt_reconfigure, please refer to:
https://github.com/ros-visualization/rqt_reconfigure/commit/3752328747b6bae95db44f432c016df27362c88e

----------------Default Parameter Description------------------------
For both ROS1 and ROS2, the default configuration is read from the Windows host PC output configuration file xintan.xtcfg. If this configuration file or other parameters are missing, dynamic parameter files will be read (ROS1 uses xtsdk_ros1.cfg, and ROS2 uses xtsdk_ros2.yaml).

----------------Dynamic Parameter Description------------------------
image_type (xtsdk output):
0: Depth Distance Image
1: Depth Image + Signal Amplitude Image
2: Grayscale Image
3: Point Cloud Output
4: Point Cloud + Signal Strength Output
camera/hdr_mode: (HDR mode)
0: Off
1: On
integration_time_tof_1: Integration time 1 value, unit: us, range: 0~2000
integration_time_tof_2: Integration time 2 value, unit: us, range: 0~2000 (valid when HDR is on)
integration_time_tof_3: Integration time 3 value, unit: us, range: 0~2000 (valid when HDR is on)
freq1: Frequency corresponding to integration time 1
freq2: Frequency corresponding to integration time 2
freq3: Frequency corresponding to integration time 3
integration_time_gray: Grayscale integration time value, unit: us, range: 0~20000 (for measuring ambient light)
min_amplitude: Minimum effective signal strength value, point position LBS, range: 0~1000
frequency_modulation: (TOF adjustment frequency)
0: 12MHz
1: 6MHz
----------------Topic Description------------------------
/xtsdk_ros/xtsdk_node/amplitude_image_raw: Signal amplitude, grayscale image Image Topic
/xtsdk_ros/xtsdk_node/distance_image_raw: Depth image Image Topic
/xtsdk_ros/xtsdk_node/gray_image_raw: Depth image Image Topic
/xtsdk_ros/xtsdk_node/points: Point cloud topic
-----------------PCD to BAG--------------------------
bash
./pcd_to_bag_converter "your_pcd_path" "your_bag_file" "topic_name"