# Change List

---
## == 2.0.0
create from 1.1.2

---
## == 2.0.1
- 适配2.x固件 
- 更新README

--
## == 2.0.2
- 修复32位距离信息赋值为16位

--
## == 2.0.3
- 增加pybinder

--
## == 2.0.4
- 优化linux cmakelist
- 优化example参数读取，配置文件加入freq4 以及 int4

--
## == 2.0.5
- 增加5档积分时间
- 优化python pybind cmaklist
- 优化通信

--
## == 2.0.6
- 增加读取xbin文件 api
- 增加 example_play示例用于读取xbin文件

--
## == 2.0.7
- 增加固件升级api
- 增加 sdk_example_ufw示例用于升级固件文件
- frame中加入 frame_label字段
- 增加doUdpFrameData 传递frame_label接口功能

--
## == 2.0.8
- 修复通信连接时发生的未收到事件回调问题

--
## == 2.0.9
- 增加录制xbin功能以及example
- 在frame中加入设备型号以及测量频率的对应反射率因子

--
## == 2.1.0
- 将cut_corner上限改为200
- frame中 反射率因子改变回退，待标定后进行更新
- 示例中适配配置文件，加入 cut_corner


--
## == 2.1.1
- 在frame中加入设备型号以及每档测量频率的对应反射率因子
- 将反射率类型改为float

--
## == 2.1.2
- merge dust分支到main
