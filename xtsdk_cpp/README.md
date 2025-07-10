

# **xtsdk_cpp V2.X**
适配雷达固件2.x版本，sdk更新2.x.x版本，兼容1.x固件版本

## News
- `2025.01.06` : 
	1. 更新多频功能，即三档积分时间可调整相应频率设定  bool setMultiModFreq(ModulationFreq freqType1, ModulationFreq freqType2, ModulationFreq freqType3, ModulationFreq freqType4 = FREQ_24M); 
	2. 固件2.20版本以上，可读取雷达内部存储的关键参数 bool getDevConfig(RespDevConfig &devConfig); 
	3. 通过api可于非灰度模式下接收灰度图 bool setAdditionalGray(const uint8_t &on);
	4. 增加灰尘滤波下移动物体优化功能以及近距离物体点云优化 void setPostProcess(const float &dilation_pixels, const uint8_t &mode, const double &mode_th);

# SDK说明

本代码为机密资料
## 简介
XTSDK 提供通过封装了对雷达交互的通讯过程(通讯、封包、深度转点云)，以易用的API方式提供编程接口，方便开发者快速集成开发。

## 文件列表
- xtsdk.h    SDK的用户接口API
- frame.h    帧的结构定义， 包含像素数据、点云数据、图像尺寸、时间、温度、类型等
- xtdaemon.h   SDK 多线程
- communication.h 通讯接口定义
- communicationNet.*  以太网通讯实现
- communicationUsb.*  Usb通讯实现
- utils.* 工具类
- cartesianTransform.*  深度转点云实现
- baseFilter.*  基础滤波实现

## 编译环境
### windows:
    语言：C++ 17以上
    依赖库： boost_1_78_0以上
    编译配置： cmake 3.21及以上
    编译器： msvc 2019、mingw8.10
    
    如下是在msvc2019windows环境下所需的第三方依赖库和编译工具
    - cmake： cmake-3.23.3-windows-i386.zip
      - 下载地址 https://github.com/Kitware/CMake/releases/download/v3.23.3/cmake-3.23.3-windows-i386.zip
    - 编译器：
      - visulstudio网站下载 Build Tools for Visual Studio 2019 (version 16.0)   x64版本
      - 下载后安装，都使用默认选项
      
    - boost库：boost_1_79_0-msvc-14.2-64.exe
      - 下载链接 https://boostorg.jfrog.io/artifactory/main/release/1.79.0/binaries/boost_1_79_0-msvc-14.2-64.exe
      
    - opencv库：opencv-4.6.0-vc14_vc15.exe
      - 下载链接 https://opencv.org/releases/
      - https://sourceforge.net/projects/opencvlibrary/files/4.6.0/opencv-4.6.0-vc14_vc15.exe/download
      
    - pcl库：PCL-1.12.1-AllInOne-msvc2019-win64 .exe
      - 下载链接 https://github.com/PointCloudLibrary/pcl/releases/download/pcl-1.12.1-rc1/PCL-1.12.1-rc1-AllInOne-msvc2019-win64.exe
    - 编译：cmake-gui.exe
      - 可以使用cmake-gui.exe 指定xtsdk_cpp目录，然后指定输出目录、选择编译器进行编译

### linux
    x86: ubuntu 18.04及以上版本         
    aarch64: ubuntu 20.04及以上版本 （aarch64 20.04以下版不支持滤波算法）
    - Boost
        sudo apt install libboost-all-dev
    
    - PCL
        如果不做点云显示，可以不依赖  
        sudo apt install libpcl-dev
    - opencv
        点云变换使用到cv对内参进行处理，滤波使用到cv的中值滤波处理。: sudo apt install libopencv-dev
    

    
## 编译方法
- linux:
    cd ~/.xtsdk_cpp
    mkdir build && cd build
    cmake ..    
    make 
    示例默认生成于~/.xtsdk_bin_release
- windows:
    cd C:\Users\YourUsername\.xtsdk_cpp
    mkdir build && cd build
    cmake .. -G "Visual Studio 16 2019"  # 2019版本的Visual Studio    
    make 
    示例默认生成于C:\Users\YourUsername\xtsdk_bin_release


# SDK 接口API说明
## 状态码：

### SDK状态
    STATE_UNSTARTUP     sdk未开启
    STATE_PORTOPENING   尝试通信端口
    STATE_TXRX_VERIFYING 通信端口已打开, 收发验证通信中
    STATE_UDPIMGOK      收到UDP数据，但TCP尝试连接中
    STATE_CONNECTED     设备已连接(通信正常)
    STATE_UNKNOW        未知状态，理论不会出现此状态

#### 雷达设备状态
    详见 xtsdk.h中 DevStateCode定义

### 命令返回状态
    详见 xtsdk.h中 CmdRespCode 定义

### Event数据类型：
    sdkState    sdk状态已改变
    devState    设备状态已改变


## 回调函数
void setCallback(std::function<void(const std::shared_ptr\<CBEventData> &) >  eventcallback, std::function<void(const std::shared_ptr\<Frame> & ) >  imgcallback);

- 设置用于接收sdk回调的两个回调函数
  - eventcallback： 用户接收sdk事件、设备上报的信息、日志等
  - imgcallback： 用户接收 设备的深度、灰度、点云帧

## 芯探激光雷达SDK数据解说
  芯探的激光雷达是Flash类型的，整个sensor全局曝光，所有点都是相同时间点，没有Ring通道的概念， 
	我们输出的点云格式是XYZI，整帧一个时间戳。

	点云的坐标系是就是芯探的雷达坐标系，雷达是坐标系原点。

	相机数据都在imgCallback 中传入的frame 对象中, 
		frame里数据说明：  
			uint16_t width; //数据分辨率宽
			uint16_t height;//数据分辨率高
			uint64_t timeStampS;//帧的时间:秒
			uint32_t timeStampNS;//帧的时间:纳秒
			uint16_t temperature;//sensor 温度
			uint16_t vcseltemperature;//灯板温度
			std::vector<uint16_t> distData;//排序后的距离数据, 深度图从左上角到右下角依次排列所有像素，有效数值是0~964000，964000以上的数值表示此像素无效
			std::vector<uint16_t> amplData;//排序后的信号强度数据, 信号强度从左上角到右下角依次排列所有像素，有效数值是0~2039，64000以上的数值表示此像素无效
			std::vector<XtPointXYZI> points;//点云数据，无序点云，排列方式和distData及amplData数据一样，按像素从左上角到右下角依次排列所有像素
					points里的点数据格式是 XYZI，例如第10行第20列的点的z方向数值是 points[width*10 + 20].z，已去畸变

		***NOTE***
			std::vector<uint16_t> distData 为滤波过后深度数组 rawdistData 为滤波过滤前深度数组，上述深度数据均未去畸变
                        ## C++ 示例简述
                        芯探SDK提供了两个示例程序源码，以方便用户快速开发应用软件
                        - sdk_example：
                            实现连接设备进行测量的最基本的 源码，只在收到图像时做日志屏显，不做绘图。
                        
                        - sdk_example_pcl：
                            实现从设备获取数据，且用PCL 做点云显示。
                        
                        - sdk_example_play：
                            读取xbin文件 显示点云
                            
                        - sdk_example_record：
                            录制xbin文件
                            
                        - sdk_example_ufw：
                            在线升级固件
                        
                                
                        ***NOTE*** 
                        
                        sdk_example_pcl 与 sdk_example_play 依赖pcl以及vtk进行渲染，在arm环境或者某些x86环境中，如果运行后发生coredump 
                        1. 请检查本机是否支持硬件gpu渲染或者检查opengl与vtk以及pcl库的兼容性，在arm中建议使用稳定版pcl 1.14以及vtk 9.1进行渲染
                        2. 如1无法解决，在运行前，终端输入export LIBGL_ALWAYS_SOFTWARE=1 直接使用软件渲染，可能会增加cpu占用 影响点云帧率，非数据帧降低
                        *********
                        
                        
                        sdk_example_pcl：
                                运行参数1：雷达地址
                                运行参数2：1 为使用本地配置文件xtsdk_cpp/sdk_example/cfg/xintan.xtcfg 忽略雷达保存的参数
                                        0 为使用雷达保存的参数 忽略本地配置文件xtsdk_cpp/sdk_example/cfg/xintan.xtcfg
                                例如 ./sdk_exampe_pcl 192.168.0.101 0
                        
                        sdk_example_play：
                                复制xbin录像文件夹a至可执行文件同级目录 
                                例如 ./sdk_exampe_play a/
                                
                        sdk_example_record： 
                                运行参数1：雷达地址
                                运行参数2：保存xbin文件夹路径
                                运行参数3：1 为使用本地配置文件xtsdk_cpp/sdk_example/cfg/xintan.xtcfg 忽略雷达保存的参数
                                        0 为使用雷达保存的参数 忽略本地配置文件xtsdk_cpp/sdk_example/cfg/xintan.xtcfg
                                例如 ./sdk_exampe_play 192.168.0.101 ./ 0
                                
                        sdk_example_ufw：
                                复制固件*.bin文件至可执行文件同级目录 
                                运行参数1：雷达地址
                                运行参数2：*.bin路径
                                例如 ./sdk_exampe_ufw 192.168.0.101 *.bin    
                                手动重启设备
                        
                        
                        
                        ******************************************************************************
                        linux环境下 如果使用串口通信，雷达地址应为/dev/ttyACM+数字 一般为 /dev/ttyACM0
                        步骤如下：
                                1. ls /dev/tty* 找到/dev/ttyACM+数字 如果不存在 表明linux中，雷达通过串口未正确连接，请检查连接   
                                2. sudo chmod 777 /dev/ttyACM+数字    一般为 /dev/ttyACM0
                                3. ./sdk_exampe_pcl /dev/ttyACM+数字 0
                                   其中升级固件时 需要 sudo ./sdk_exampe_ufw /dev/ttyACM+数字 *.bin    
                        ******************************************************************************			

## Sdk运行API
- bool setConnectIpaddress(std::string ipAddress);  
  如果用网络连接，使用这个API设置要连接设备的ip地址(如 "192.168.0.101")

- bool setConnectSerialportName(std::string serialportName);  
​	如果用USB连接，使用这个API设置要连接设备的COM口地址(如 "COM2")

- void startup();  
​	启动sdk运行

- void shutdown();  
​	断开设备连接

- bool isconnect();  
​	设备连接是否成功

- bool isUsedNet();   
  设备连接是否通过网络

- SdkState getSdkState();  
​	获取SDK状态

- SdkState getDevState();  
​	获取设备状态

- std::string getStateStr();  
​	获取SDK 状态字符串

- int getfps();  
    获取SDK计算的帧率
- bool setSdkReflectiveFilter(const float &threshold_min, const float &threshold_max);
  设置sdk中的反射率滤波    

- bool setSdkKalmanFilter(uint16_t factor, uint16_t threshold);  
  设置sdk中的卡尔曼滤波

- bool setSdkEdgeFilter(uint16_t threshold);  
  设置sdk中的飞点滤波

- bool setSdkMedianFilter(uint16_t size);  
    设置sdk中的中值滤波

- void setPostProcess(const float &dilation_pixels, const uint8_t &mode);
   设置近距离点云优化以及移动物体优化

- bool clearAllSdkFilter();  
  清除SDK中所有的滤波设置

- bool setSdkCloudCoordType(ClOUDCOORD_TYPE type); 
  设置sdk中输出点云的坐标系

## 命令相关 API

- bool testDev();  
​	测试设备命令交互是否通

- bool start(ImageType imgType, bool isOnce = false);  
​	指定期望的图像类型 进行测量, isOnce 是否单次获取

- bool stop();  
​	让设备停止测量

- bool getDevInfo(RespDevInfo & devInfo);  
​	获取设备信息，数据结构见xtsdk.h定义

- bool getDevConfig(RespDevConfig & devConfig);  
​	获取设备设置信息，数据结构见xtsdk.h定义

- bool setIp(std::string ip, std::string mask, std::string gate);  
​	设置设备的ip地址相关  
​	ip 如 “192.168.0.101”  
​	mask 如”255.255.255.0”  
​	gate 如 “192.168.0.1”

- bool setFilter(uint16_t temporal_factor, uint16_t temporal_threshold, uint16_t edgefilter_threshold);  
​	设置设备内基本滤波参数  
​	temporal_factor   卡尔曼 因子 ，最大1000，通常设定300  
​	temporal_threshold  卡尔曼 阈值，最大2000， 通常设定300  
​	edgefilter_threshold  飞点 阈值，最大2000， 通常设定 0

- bool setIntTimesus(uint16_t timeGs, uint16_t time1, uint16_t time2, uint16_t time3);​	  
    设置积分时间 单位 us  
​	timeGs   测量灰度时的积分时间  
​	time1     第一次积分时间  
​	time2     第二次积分时间，开HDR时才有效，设为0 即关闭此次曝光  
​	time3     第三次积分时间，HDR 为HDR_TAMPORAL   模式时才有效，设为0 即关闭此次曝光

- bool setMinAmplitude(uint16_t minAmplitude);   
​	设置有效的信号幅度下限, 通常设定为50~100

- bool setHdrMode(HDRMode mode);  
​	设置HDR 类型，mode可配置为  
​	HDR_OFF    关闭HDR  
​	HDR_TAMPORAL   时域模式  

- bool resetDev();  
​	重启设备

- bool setModFreq(ModulationFreq freqType);  
​	设置设备调整频率  
​	freqType：  FREQ_12M，FREQ_6M

- bool setRoi(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);  
​	设置ROI区域

- bool setUdpDestIp(std::string ip, uint16_t port =7687);  
  设置UDP目标IP地址

- bool setMaxFps(uint8_t maxfps);  
  设置设备测量的最快帧率，(HDR模式配置为实际要的帧率的多倍(几个积分时间))

- bool getLensCalidata(CamParameterS & camparameter);  
  获取镜头内参

- bool getDevConfig(RespDevConfig &devConfig); 
  获取设备设置信息

- bool setAdditionalGray(const uint8_t &on);
  设置是否获取灰度图

- bool setMultiModFreq(ModulationFreq freqType1, ModulationFreq freqType2, ModulationFreq freqType3, ModulationFreq freqType4 = FREQ_24M); 
  设置三档积分时间对应频率

- bool setBinningV(uint8_t flag);
  0为不设定binningv 1为设定binningv

- bool getImuExtParamters(ExtrinsicIMULidar &imuparameters, uint8_t flag = 1);
  获取imu to lidar外参 0为获取默认外参，1为获取标定外参，
  如果需要setTransMirr需要设定 必须在设定之后 获取外参
