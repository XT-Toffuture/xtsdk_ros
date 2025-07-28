# xtsdk_ros
本代码为机密资料

此ros包支持 x86: ros1 melodic 和 ros2 foxy humble jazzy版本  
	aarch64: ros1 noetic 和 ros2 foxy humble jazzy版本  
芯探ros功能是基于xtsdk_cpp实现的，xtsdk_cpp详见xtsdk_cpp子目录下readme.md文档

## ros版本选择
	将 xtsdk_ros 拷贝至工作空间的 src 目录下  
	进入xtsdk_ros目录  
	chmod +x selros.sh  
	./selros.sh 1     为选择ros1 版本  
	./selros.sh 2     为选择ros2 版本


## ip地址修改
	使用时请设置修改默认连接的ip地址
### ROS1	
	使用时请先修改 cfg/xtsdk_ros1.cfg里
	如果ip连接
		gen.add("usb_com",           bool_t, 0,  	"SET USB",  False)
		gen.add("connect_address",     str_t,  0,  	"connect address ip/serial", "192.168.2.101")   里的ip地址,此为激光雷达地址
	如果usb连接
		sudo chmod 777 /dev/ttyACM0
		gen.add("usb_com",           bool_t, 1,  	"SET USB",  True)

	然后进行编译

### ROS2
	使用时请先修改 cfg/xtsdk_ros2.yaml里
	如果ip连接
		usb_com: false
		connect_address: 192.168.0.101  里的ip地址,此为激光雷达地址
	如果usb连接
		sudo chmod 777 /dev/ttyACM0
		usb_com: true


	
	

## 编译环境
	支持 ubuntu 18.04  或 ubuntu 20.04
	支持 ros melodic  或 ros2 foxy

## 依赖库：
- boost： 根据自己的Boost路径，配置下xtsdk的cmakelist.txt文件
- opencv: 请安装opencv库： sudo apt install libopencv-dev


# ----------------ROS1 -----------------------------------
## 编译
	ros1 环境load： source /opt/ros/melodic/setup.bash  
	catkin_make install

## 运行
	source install/setup.bash  
	roslaunch xtsdk_ros xtsdk_ros1.launch

## 调参
	初始读取参数过程中，雷达固件2.20以上会自动更新设备参数至调参界面，否则读取 xtsdk_ros/cfg/xintan.xtcfg


# ----------------ROS2-----------------------------------
## 编译
	ros2 环境load： source /opt/ros/foxy/setup.bash
	colcon build


## 运行
	source install/setup.bash
	ros2 launch xtsdk_ros xtsdk_ros2.py

## 调参
	通过rqt_reconfigure 进行参数调试，其中雷达固件2.20以上时，会自动更新参数至调参界面，否则读取 xtsdk_ros/cfg/xintan.xtcfg
	
	##NOTE##
 	在ros2中，雷达固件为2.20以后的版本 获取雷达保存参数 并使用rqt_reconfigure gui时 如果无法实时更新界面 请参考
	https://github.com/ros-visualization/rqt_reconfigure/commit/3752328747b6bae95db44f432c016df27362c88e

# ----------------默认参数说明------------------------
ros1，ros2示例默认读取来自windows上位机输出的配置文件xintan.xtcfg,
若无此配置文件或者其他参数则读取动态参数文件（ros1 为 xtsdk_ros1.cfg, ros2 为 xtsdk_ros2.yaml）

# ----------------动态参数说明------------------------
- image_type ( xtsdk输出   0：深度距离图 1：深度图+信号幅度图 2：灰度图  3：点云输出  4：点云+信号强度输出)
- camera/hdr_mode   ( hdr模式     0:关闭 1：开启)
- integration_time_tof_1	(积分时间1的数值，单位us，范围0~2000)
- integration_time_tof_2	(积分时间2的数值，单位us，范围0~2000， hdr开启才有效)
- integration_time_tof_3	(积分时间3的数值，单位us，范围0~2000,  hdr开启才有效)
- freq1   (积分时间1对应频率）
- freq2   (积分时间2对应频率）
- freq3   (积分时间3对应频率）
- integration_time_gray   (灰度积分时间的数值，单位us，范围0~20000，测环境光使用)
- min_amplitude		    (最小有效的信号强度值，点位LBS，范围0~1000)
- frequency_modulation	(tof调整频率， 0：12Mhz   1：6Mhz)




# ----------------Topic说明------------------------

- /xtsdk_ros/xtsdk_node/amplitude_image_raw   信号幅度、灰度图 Image Topic
- /xtsdk_ros/xtsdk_node/distance_image_raw	深度图 Image Topic
- /xtsdk_ros/xtsdk_node/gray_image_raw	深度图 Image Topic
- /xtsdk_ros/xtsdk_node/points		点云 topic

# -----------------pcd转bag--------------------------
- ./pcd_to_bag_converter "your_pcd_path" "your_bag_file" "topic_name"
