#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <xtsdk_ros/xtsdk_ros1Config.h>

#include <mutex>
#include <condition_variable>
#include <atomic>
#include <csignal>
#include <thread>
std::condition_variable cv; // 条件变量，用于等待和通知
#include "xtsdk.h"
#include "para_ros.h"

using namespace XinTan;
para_ros para_curr;
para_ros para_set;
ExtrinsicIMULidar e_imu_lidar;
static std::string connect_address_cur;

ros::Publisher distanceImagePublisher;
ros::Publisher amplitudeImagePublisher;
ros::Publisher grayImagePublisher;

ros::Publisher cameraInfoPublisher;
ros::Publisher pointCloud2Publisher;
ros::ServiceServer cameraInfoService;

sensor_msgs::CameraInfo cameraInfo;

uint8_t check_count = 0;
XtSdk *xtsdk;

std::atomic<bool> keepRunning(true);
uint8_t is_connected = 0;
uint8_t is_connected_once = 0;
bool get_config_success = false;
std::mutex mtx; // 互斥锁，用于保护共享数据
bool update_once = false;
double fwversionf = 0.0;
xtsdk_ros::xtsdk_ros1Config config;
xtsdk_ros::xtsdk_ros1Config config_device;

// 参数设置

void update_config_from_dev(xtsdk_ros::xtsdk_ros1Config &config)
{
    config.freq1 = config_device.freq1;
    config.freq2 = config_device.freq2;
    config.freq3 = config_device.freq3;
    config.freq4 = config_device.freq4;
    config.freq = config_device.freq;

    config.int1 = config_device.int1;
    config.int2 = config_device.int2;
    config.int3 = config_device.int3;
    config.int4 = config_device.int4;
    config.intgs = config_device.intgs;

    config.imgType = config_device.imgType;
    config.HDR = config_device.HDR;
    config.maxfps = config_device.maxfps;
    config.minLSB = config_device.minLSB;
}
void setParameters()
{
    ROS_INFO("set parameters...");
    if (para_set.lidar_setting_.usb_com_name !=
        para_curr.lidar_setting_.usb_com_name)
    {
        std::cout << " seting USB COM name:  "
                  << para_curr.lidar_setting_.usb_com_name << std::endl;
        para_set.lidar_setting_.usb_com_name =
            para_curr.lidar_setting_.usb_com_name;
        if (para_set.lidar_setting_.usb_com)
        {
            if (para_set.lidar_setting_.usb_com_name != "")
            {
                xtsdk->stop();
                xtsdk->setConnectIpaddress(
                    para_set.lidar_setting_.usb_com_name); // linux
                xtsdk->start((ImageType)para_set.lidar_setting_.imgType);
                ROS_INFO("connect_com %s",
                         para_set.lidar_setting_.usb_com_name.c_str());
            }
            else
            {
                ROS_WARN("com_name is null");
            }
        }
    }
    if (para_set.lidar_setting_.usb_com != para_curr.lidar_setting_.usb_com)
    {
        std::cout << " seting USB COM:  " << para_curr.lidar_setting_.usb_com
                  << std::endl;
        para_set.lidar_setting_.usb_com = para_curr.lidar_setting_.usb_com;
        if (para_set.lidar_setting_.usb_com)
        {
            if (para_set.lidar_setting_.usb_com_name != "")
            {
                xtsdk->stop();
                xtsdk->setConnectIpaddress(
                    para_set.lidar_setting_.usb_com_name); // linux
                // xtsdk->start((ImageType)para_set.lidar_setting_.imgType);
                ROS_INFO("connect_com %s",
                         para_set.lidar_setting_.usb_com_name.c_str());
            }
            else
            {
                ROS_WARN("com_name is null");
            }
        }
        else
        {
            // xtsdk->stop();
            std::cout << "set ip: " << para_set.lidar_setting_.connect_address
                      << std::endl;
            xtsdk->setConnectIpaddress(para_set.lidar_setting_.connect_address);
            // xtsdk->start((ImageType)para_set.lidar_setting_.imgType);
        }
    }

    if (para_set.lidar_setting_.connect_address !=
        para_curr.lidar_setting_.connect_address)
    {
        if (!para_set.lidar_setting_.usb_com)
        {
            std::cout << " seting: " << para_curr.lidar_setting_.connect_address
                      << std::endl;
            para_set.lidar_setting_.connect_address =
                para_curr.lidar_setting_.connect_address;
            xtsdk->setConnectIpaddress(para_set.lidar_setting_.connect_address);
            ROS_INFO("connect_address %s",
                     para_set.lidar_setting_.connect_address.c_str());
        }
        else
        {
            ROS_WARN("USB MODE SET");
        }
    }

    if (para_set.lidar_setting_.start_stream !=
        para_curr.lidar_setting_.start_stream)
    {

        para_set.lidar_setting_.start_stream =
            para_curr.lidar_setting_.start_stream;
        if (para_set.lidar_setting_.start_stream)
        {
            // while
            // (!xtsdk->start((ImageType)para_set.lidar_setting_.imageType))
            // {
            //     ros::Duration(1).sleep();
            //     ROS_INFO("start_stream ...");
            // }

            xtsdk->start((ImageType)para_set.lidar_setting_.imgType);
        }
        else
        {
            xtsdk->stop();
        }
        ROS_INFO("start_stream %d", para_set.lidar_setting_.start_stream);
    }

    if (para_set.lidar_setting_.gray_on != para_curr.lidar_setting_.gray_on)
    {

        para_set.lidar_setting_.gray_on = para_curr.lidar_setting_.gray_on;
        if (para_set.lidar_setting_.gray_on)
        {
            xtsdk->setAdditionalGray(1);
        }
        else
        {
            xtsdk->setAdditionalGray(0);
        }
        ROS_INFO("gray_on %d", para_set.lidar_setting_.gray_on);
    }

    if (para_set.lidar_setting_.is_use_devconfig != para_curr.lidar_setting_.is_use_devconfig)
    {

        para_set.lidar_setting_.is_use_devconfig = para_curr.lidar_setting_.is_use_devconfig;
        ROS_INFO("is_use_devconfig %d", para_set.lidar_setting_.is_use_devconfig);
    }

    if (para_set.lidar_setting_.imgType != para_curr.lidar_setting_.imgType)
    {

        para_set.lidar_setting_.imgType = para_curr.lidar_setting_.imgType;
        if (para_set.lidar_setting_.start_stream)
        {
            xtsdk->start((ImageType)para_set.lidar_setting_.imgType);
        }
        else
        {
            xtsdk->stop();
        }

        ROS_INFO("image_type %d", para_set.lidar_setting_.imgType);
    }

    if (para_set.lidar_setting_.cloud_coord !=
        para_curr.lidar_setting_.cloud_coord)
    {

        para_set.lidar_setting_.cloud_coord =
            para_curr.lidar_setting_.cloud_coord;
        xtsdk->setSdkCloudCoordType(
            (ClOUDCOORD_TYPE)para_set.lidar_setting_.cloud_coord);
        ROS_INFO("cloud_coord %d", para_set.lidar_setting_.cloud_coord);
    }

    if (para_set.lidar_setting_.freq != para_curr.lidar_setting_.freq)
    {

        para_set.lidar_setting_.freq = para_curr.lidar_setting_.freq;
        if (fwversionf < 2.0)
        {
            xtsdk->setModFreq((ModulationFreq)para_set.lidar_setting_.freq);
            ROS_INFO("freq %d", para_set.lidar_setting_.freq);
        }
        else
        {
            ROS_INFO("freq not set to %d", para_set.lidar_setting_.freq);
        }
    }

    if (para_set.lidar_setting_.freq1 != para_curr.lidar_setting_.freq1)
    {

        para_set.lidar_setting_.freq1 = para_curr.lidar_setting_.freq1;
        xtsdk->setMultiModFreq((ModulationFreq)para_set.lidar_setting_.freq1,
                               (ModulationFreq)para_set.lidar_setting_.freq2,
                               (ModulationFreq)para_set.lidar_setting_.freq3,
                               (ModulationFreq)para_set.lidar_setting_.freq4);
        ROS_INFO("freq1 %d", para_set.lidar_setting_.freq1);
    }

    if (para_set.lidar_setting_.freq2 != para_curr.lidar_setting_.freq2)
    {

        para_set.lidar_setting_.freq2 = para_curr.lidar_setting_.freq2;
        xtsdk->setMultiModFreq((ModulationFreq)para_set.lidar_setting_.freq1,
                               (ModulationFreq)para_set.lidar_setting_.freq2,
                               (ModulationFreq)para_set.lidar_setting_.freq3,
                               (ModulationFreq)para_set.lidar_setting_.freq4);
        ROS_INFO("freq2 %d", para_set.lidar_setting_.freq2);
    }

    if (para_set.lidar_setting_.freq3 != para_curr.lidar_setting_.freq3)
    {

        para_set.lidar_setting_.freq3 = para_curr.lidar_setting_.freq3;
        xtsdk->setMultiModFreq((ModulationFreq)para_set.lidar_setting_.freq1,
                               (ModulationFreq)para_set.lidar_setting_.freq2,
                               (ModulationFreq)para_set.lidar_setting_.freq3,
                               (ModulationFreq)para_set.lidar_setting_.freq4);
        ROS_INFO("freq3 %d", para_set.lidar_setting_.freq3);
    }

    if (para_set.lidar_setting_.freq4 != para_curr.lidar_setting_.freq4)
    {

        para_set.lidar_setting_.freq4 = para_curr.lidar_setting_.freq4;
        xtsdk->setMultiModFreq((ModulationFreq)para_set.lidar_setting_.freq1,
                               (ModulationFreq)para_set.lidar_setting_.freq2,
                               (ModulationFreq)para_set.lidar_setting_.freq3,
                               (ModulationFreq)para_set.lidar_setting_.freq4);
        ROS_INFO("freq4 %d", para_set.lidar_setting_.freq4);
    }

    if (para_set.lidar_setting_.HDR != para_curr.lidar_setting_.HDR)
    {

        para_set.lidar_setting_.HDR = para_curr.lidar_setting_.HDR;
        xtsdk->setHdrMode((HDRMode)para_set.lidar_setting_.HDR);
        ROS_INFO("hdr_mode %d", para_set.lidar_setting_.HDR);
    }

    if ((para_set.lidar_setting_.int1 != para_curr.lidar_setting_.int1) ||
        (para_set.lidar_setting_.int2 != para_curr.lidar_setting_.int2) ||
        (para_set.lidar_setting_.int3 != para_curr.lidar_setting_.int3) ||
        (para_set.lidar_setting_.int3 != para_curr.lidar_setting_.int4) ||
        (para_set.lidar_setting_.intgs != para_curr.lidar_setting_.intgs))
    {
        para_set.lidar_setting_.int1 = para_curr.lidar_setting_.int1;
        para_set.lidar_setting_.int2 = para_curr.lidar_setting_.int2;
        para_set.lidar_setting_.int3 = para_curr.lidar_setting_.int3;
        para_set.lidar_setting_.int4 = para_curr.lidar_setting_.int4;
        para_set.lidar_setting_.intgs = para_curr.lidar_setting_.intgs;

        xtsdk->setIntTimesus(para_set.lidar_setting_.intgs, para_set.lidar_setting_.int1,
                             para_set.lidar_setting_.int2, para_set.lidar_setting_.int3,
                             para_set.lidar_setting_.int4);

        ROS_INFO("integration_time0 %d", para_set.lidar_setting_.int1);
        ROS_INFO("integration_time1 %d", para_set.lidar_setting_.int2);
        ROS_INFO("integration_time2 %d", para_set.lidar_setting_.int3);
        ROS_INFO("integration_time3 %d", para_set.lidar_setting_.int4);
        ROS_INFO("integration_time_gray %d", para_set.lidar_setting_.intgs);
    }

    if (para_set.lidar_setting_.minLSB != para_curr.lidar_setting_.minLSB)
    {

        para_set.lidar_setting_.minLSB = para_curr.lidar_setting_.minLSB;
        xtsdk->setMinAmplitude(para_set.lidar_setting_.minLSB);
        ROS_INFO("min_amplitude %d", para_set.lidar_setting_.minLSB);
    }

    if (para_set.lidar_setting_.cut_corner !=
        para_curr.lidar_setting_.cut_corner)
    {

        para_set.lidar_setting_.cut_corner =
            para_curr.lidar_setting_.cut_corner;
        xtsdk->setCutCorner(para_curr.lidar_setting_.cut_corner);
        ROS_INFO("cut_corner %d", para_set.lidar_setting_.cut_corner);
    }

    if (para_set.lidar_setting_.maxfps != para_curr.lidar_setting_.maxfps)
    {
        para_set.lidar_setting_.maxfps = para_curr.lidar_setting_.maxfps;
        xtsdk->setMaxFps(para_curr.lidar_setting_.maxfps);
        ROS_INFO("maxfps %d", para_set.lidar_setting_.maxfps);
    }

    if (para_set.lidar_setting_.hmirror != para_curr.lidar_setting_.hmirror)
    {
        para_set.lidar_setting_.hmirror = para_curr.lidar_setting_.hmirror;
        xtsdk->setTransMirror(para_set.lidar_setting_.hmirror, para_set.lidar_setting_.vmirror);
        if ((para_set.lidar_setting_.hmirror && para_set.lidar_setting_.vmirror) ||
            (!para_set.lidar_setting_.hmirror && !para_set.lidar_setting_.vmirror))
        {
            xtsdk->getImuExtParamters(e_imu_lidar, 1);
        }
        ROS_INFO("hmirror %d", para_set.lidar_setting_.hmirror);
    }

    if (para_set.lidar_setting_.vmirror != para_curr.lidar_setting_.vmirror)
    {
        para_set.lidar_setting_.vmirror = para_curr.lidar_setting_.vmirror;
        xtsdk->setTransMirror(para_set.lidar_setting_.hmirror, para_set.lidar_setting_.vmirror);
        if ((para_set.lidar_setting_.hmirror && para_set.lidar_setting_.vmirror) ||
            (!para_set.lidar_setting_.hmirror && !para_set.lidar_setting_.vmirror))
        {
            xtsdk->getImuExtParamters(e_imu_lidar, 1);
        }
        ROS_INFO("vmirror %d", para_set.lidar_setting_.vmirror);
    }

    if (para_set.lidar_setting_.binningV != para_curr.lidar_setting_.binningV)
    {
        para_set.lidar_setting_.binningV = para_curr.lidar_setting_.binningV;
        xtsdk->setBinningV(para_set.lidar_setting_.binningV);
        ROS_INFO("binningV %d", para_set.lidar_setting_.binningV);
    }

    // if (para_set.lidar_ros_.frame_id != para_curr.lidar_ros_.frame_id)
    // {
    //     para_set.lidar_ros_.frame_id = para_curr.lidar_ros_.frame_id;
    //     ROS_INFO("frame_id %s", para_set.lidar_ros_.frame_id.c_str());
    // }
    // if (para_set.lidar_ros_.topic_name != para_curr.lidar_ros_.topic_name)
    // {
    //     para_set.lidar_ros_.topic_name = para_curr.lidar_ros_.topic_name;

    //     ROS_INFO("topic_name %s", para_set.lidar_ros_.topic_name.c_str());
    // }
}

void updateConfig(xtsdk_ros::xtsdk_ros1Config &config, uint32_t level)
{
    if (!update_once)
    {
        update_once = true;
        std::cout << "update_once" << std::endl;
        return;
    }
    para_curr.lidar_setting_.start_stream = config.start_stream;
    para_curr.lidar_setting_.imgType = config.imgType;
    para_curr.lidar_setting_.minLSB = config.minLSB;
    para_curr.lidar_setting_.HDR = config.HDR;
    para_curr.lidar_setting_.int1 = config.int1;
    para_curr.lidar_setting_.int2 = config.int2;
    para_curr.lidar_setting_.int3 = config.int3;
    para_curr.lidar_setting_.int4 = config.int4;
    para_curr.lidar_setting_.intgs = config.intgs;
    para_curr.lidar_setting_.freq = config.freq;
    para_curr.lidar_setting_.freq1 = config.freq1;
    para_curr.lidar_setting_.freq2 = config.freq2;
    para_curr.lidar_setting_.freq3 = config.freq3;
    para_curr.lidar_setting_.freq4 = config.freq4;
    para_curr.lidar_setting_.connect_address = config.connect_address;
    para_curr.lidar_setting_.cut_corner = config.cut_corner;
    para_curr.lidar_setting_.maxfps = config.maxfps;
    para_curr.lidar_setting_.cloud_coord = config.cloud_coord;
    para_curr.lidar_setting_.usb_com = config.usb_com;
    para_curr.lidar_setting_.usb_com_name = config.usb_com_name;
    para_curr.lidar_setting_.gray_on = config.gray_on;
    para_curr.lidar_setting_.hmirror = config.hmirror;
    para_curr.lidar_setting_.vmirror = config.vmirror;
    para_curr.lidar_setting_.binningV = config.binningV;
    para_curr.lidar_setting_.is_use_devconfig = config.is_use_devconfig;

    // para_curr.lidar_ros_.frame_id = config.frame_id;
    // para_curr.lidar_ros_.topic_name = config.topic_name;
    std::cout << "curr com: " << para_curr.lidar_setting_.usb_com
              << " set com: " << para_set.lidar_setting_.usb_com << std::endl;
    std::cout << "curr add: " << para_curr.lidar_setting_.connect_address
              << " set add: " << para_set.lidar_setting_.connect_address
              << std::endl;
    setParameters();
}

bool setCameraInfo(sensor_msgs::SetCameraInfo::Request &req,
                   sensor_msgs::SetCameraInfo::Response &res)
{
    req.camera_info.width = 320;
    req.camera_info.height = 240;

    cameraInfoPublisher.publish(req.camera_info);

    res.success = true;
    res.status_message = "";
    return true;
}

void updateCameraInfo(std::shared_ptr<sensor_msgs::CameraInfo> ci)
{
    cameraInfo.width = ci->width;
    cameraInfo.height = ci->height;
    cameraInfo.roi.x_offset = 0; // ci->roiX0;
    cameraInfo.roi.y_offset = 0; // ci->roiY0;
    cameraInfo.roi.width = 320;  // ci->roiX1 - ci->roiX0;
    cameraInfo.roi.height = 240; // ci->roiY1 - ci->roiY0;
}

void setAmpType_reflect(std::string devsn)
{

    uint16_t ampnormalizedv = 20;
    XByteArray data = {static_cast<uint8_t>(20),
                       static_cast<uint8_t>(ampnormalizedv >> 8),
                       static_cast<uint8_t>(ampnormalizedv & 0x00ff)};
    XByteArray respdata;
    xtsdk->customCmd(202, data, respdata);
}

void initialise(const std::string &topic_name)
{
    ros::NodeHandle nh("~");

    nh.getParam("start_stream", para_set.lidar_setting_.start_stream);
    nh.getParam("image_type", para_set.lidar_setting_.imgType);
    nh.getParam("hdr_mode", para_set.lidar_setting_.HDR);
    nh.getParam("integration_time_tof_1", para_set.lidar_setting_.int1);
    nh.getParam("integration_time_tof_2", para_set.lidar_setting_.int2);
    nh.getParam("integration_time_tof_3", para_set.lidar_setting_.int3);
    nh.getParam("integration_time_tof_4", para_set.lidar_setting_.int4);
    nh.getParam("integration_time_gray", para_set.lidar_setting_.intgs);
    nh.getParam("freq", para_set.lidar_setting_.freq);
    nh.getParam("freq1", para_set.lidar_setting_.freq1);
    nh.getParam("freq2", para_set.lidar_setting_.freq2);
    nh.getParam("freq3", para_set.lidar_setting_.freq3);
    nh.getParam("freq3", para_set.lidar_setting_.freq4);
    nh.getParam("min_amplitude", para_set.lidar_setting_.minLSB);
    nh.getParam("connect_address", para_set.lidar_setting_.connect_address);
    nh.getParam("cutconer", para_set.lidar_setting_.cut_corner);
    nh.getParam("usb_com", para_set.lidar_setting_.usb_com);
    nh.getParam("usb_com_name", para_set.lidar_setting_.usb_com_name);

    // advertise publishers
    distanceImagePublisher =
        nh.advertise<sensor_msgs::Image>("distance_image_raw", 1000);
    amplitudeImagePublisher =
        nh.advertise<sensor_msgs::Image>("amplitude_image_raw", 1000);
    grayImagePublisher = nh.advertise<sensor_msgs::Image>("gray_image_raw", 1000);
    pointCloud2Publisher =
        nh.advertise<pcl::PointCloud<pcl::PointXYZI>>(topic_name, 100);
    cameraInfoPublisher =
        nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1000);

    // advertise services
    cameraInfoService = nh.advertiseService("set_camera_info", setCameraInfo);

    ROS_INFO("xtsdk ros1 initialise end");
}

//==========================================================================
void imgCallback(const std::shared_ptr<Frame> &frame)
{
    if (is_connected == 0)
    {
        ROS_INFO("not connected to lidar, skip image");
        return;
    }
    std::vector<uint32_t> distance_tmp;
    std::vector<uint16_t> amplitude_tmp;
    std::vector<uint16_t> gray_tmp;
    check_count++;
    bool check_flag = check_count > 10 ? true : false;
    // {
    uint64_t sec = frame->timeStampS;

    if (frame->timeStampType == 2)
    {
        sec -= 37;
        if (frame->timeStampState == 0)
        {
            if (check_flag)
            {
                ROS_ERROR("SYNC LOST");
                check_count = 0;
            }
        }
        else
        {
            if (check_flag)
            {
                ROS_INFO("SYNC");
                check_count = 0;
            }
        }
    }
    else
    {
        if (check_flag)
        {
            ROS_WARN("NO SYNC");
            check_count = 0;
        }
    }

    const size_t nPixel = frame->width * frame->height;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZI>());
    cloud->header.frame_id = para_set.lidar_ros_.frame_id;
    cloud->header.stamp =
        pcl_conversions::toPCL(ros::Time(sec, frame->timeStampNS));
    cloud->width = static_cast<uint32_t>(frame->width);
    cloud->height = static_cast<uint32_t>(frame->height);
    cloud->is_dense = false;
    cloud->points.resize(nPixel);
    distance_tmp.resize(nPixel);
    amplitude_tmp.resize(nPixel);
    gray_tmp.resize(nPixel);
    // std::cout << "amplData size " << frame->amplData.size() << std::endl;
#pragma omp parallel for private(y)

    for (int i = 0; i < (int)nPixel; i++)
    {
        if (frame->hasPointcloud)
        {
            pcl::PointXYZI &p = cloud->points[i];
            p.x = frame->points[i].x;
            p.y = frame->points[i].y;
            p.z = frame->points[i].z;
            p.intensity = frame->amplData[i] > AMPLITUDE_ABNORMAL
                              ? 0
                              : frame->amplData[i];
        }

        distance_tmp[i] =
            frame->distData[i] > DEPTH_ABNORMAL ? 0 : frame->distData[i];
        amplitude_tmp[i] =
            frame->amplData[i] > AMPLITUDE_ABNORMAL ? 0 : frame->amplData[i];
        if (frame->info.imageflags & IMG_GS16)
        {
            gray_tmp[i] = frame->grayscaledata[i];
        }
    }

    std::cout << "img: " + std::to_string(frame->frame_id) << std::endl;

    pointCloud2Publisher.publish(cloud);
    if (frame->dataType == Frame::DISTANCE ||
        frame->dataType == Frame::AMPLITUDE)
    {

        sensor_msgs::Image imgDistance;

        // 填充基本信息
        imgDistance.header.stamp.sec = sec;
        imgDistance.header.stamp.nsec = frame->timeStampNS;
        imgDistance.header.frame_id = para_set.lidar_ros_.frame_id;
        imgDistance.height = static_cast<uint32_t>(frame->height);
        imgDistance.width = static_cast<uint32_t>(frame->width);
        imgDistance.encoding = sensor_msgs::image_encodings::MONO16; // 使用 16 位单通道图像编码
        imgDistance.step = imgDistance.width * 2;                    // 每行的字节数，因为每个像素是 2 字节
        imgDistance.is_bigendian = 1;                                // 设置字节序，若需要大端序，则为 1，否为 0

        // 清空现有的数据，准备填充新的数据
        imgDistance.data.clear();

        // 将 distance_tmp 中的数据转换为 16 位图像数据
        for (size_t i = 0; i < distance_tmp.size(); ++i)
        {
            uint32_t value = distance_tmp[i];

            // 提取低 2 字节（按小端序）并转换为 uint8_t
            uint8_t low_byte = static_cast<uint8_t>(value & 0xFF);
            uint8_t high_byte = static_cast<uint8_t>((value >> 8) & 0xFF);

            // 填充 16 位图像数据（低字节先，符合小端序）
            imgDistance.data.push_back(low_byte);
            imgDistance.data.push_back(high_byte);
        }
        distanceImagePublisher.publish(imgDistance);
    }
    if (frame->dataType == Frame::AMPLITUDE)
    {
        sensor_msgs::Image imgAmpl;
        imgAmpl.header.stamp.sec = sec;
        imgAmpl.header.stamp.nsec = frame->timeStampNS;
        imgAmpl.header.frame_id = para_set.lidar_ros_.frame_id;
        imgAmpl.height = static_cast<uint32_t>(frame->height);
        imgAmpl.width = static_cast<uint32_t>(frame->width);
        imgAmpl.encoding = sensor_msgs::image_encodings::MONO16;
        imgAmpl.step = imgAmpl.width * frame->px_size;
        imgAmpl.is_bigendian = 1;

        // imgAmpl.data.assign((uint8_t *)(frame->amplData.data()), (uint8_t
        // *)(frame->amplData.data()) + frame->amplData.size() * 2);
        imgAmpl.data.assign((uint8_t *)(amplitude_tmp.data()), (uint8_t *)(amplitude_tmp.data()) + amplitude_tmp.size() * 2);
        amplitudeImagePublisher.publish(imgAmpl);
    }
    if (frame->info.imageflags & IMG_GS16)
    {
        sensor_msgs::Image imgGray;
        imgGray.header.stamp.sec = sec;
        imgGray.header.stamp.nsec = frame->timeStampNS;
        imgGray.header.frame_id = para_set.lidar_ros_.frame_id;
        imgGray.height = static_cast<uint32_t>(frame->height);
        imgGray.width = static_cast<uint32_t>(frame->width);
        imgGray.encoding = sensor_msgs::image_encodings::MONO16;
        imgGray.step = imgGray.width * frame->px_size;
        imgGray.is_bigendian = 1;

        // imgAmpl.data.assign((uint8_t *)(frame->amplData.data()), (uint8_t
        // *)(frame->amplData.data()) + frame->amplData.size() * 2);
        imgGray.data.assign((uint8_t *)(gray_tmp.data()), (uint8_t *)(gray_tmp.data()) + gray_tmp.size() * 2);
        grayImagePublisher.publish(imgGray);
    }
}

int getMultiFreq(const int &freq_dev)
{
    int index = 0;
    switch (freq_dev)
    {
    case 0:
        index = 2;
        break;
    case 1:
        index = 0;
        break;
    case 3:
        index = 1;
        break;
    case 4:
        index = 6;
        break;
    case 7:
        index = 3;
        break;
    case 15:
        index = 4;
        break;

    default:
        std::cout << "Unknown FREQ!" << std::endl;
        index = 100;
        break;
    }
    return index;
}
int imucount = 0;
void eventCallback(const std::shared_ptr<CBEventData> &event)
{
    // std::cout << "event: " + event->eventstr + " " +
    //                  std::to_string(event->cmdid)
    //           << std::endl;

    if (event->eventstr == "sdkState")
    {
        // std::cout << "devstate= " + xtsdk->getStateStr() << std::endl;
        XTAPPLOG("devstate= " + xtsdk->getStateStr());
        if (xtsdk->isconnect() &&
            (event->cmdid == 0xfe)) // 端口打开后第一次连接上设备
        {
            xtsdk->stop();

            RespDevInfo devinfo;
            xtsdk->getDevInfo(devinfo);
            std::cout << std::endl
                      << devinfo.fwVersion.c_str() << std::endl;
            std::cout << devinfo.sn.c_str() << devinfo.chipidStr << std::endl;

            XTAPPLOG("DEV SN=" + devinfo.sn);

            int fwlen = devinfo.fwVersion.length();
            fwversionf = atof(&(devinfo.fwVersion[fwlen - 4]));
            std::cout << "fwversionf " << fwversionf << std::endl;
            if (fwversionf >= 2.20)
            {
                std::cout << "> 2.20" << std::endl;
                RespDevConfig devconfig;
                if (xtsdk->getDevConfig(devconfig))
                {
                    get_config_success = true;
                    config_device.freq1 =
                        getMultiFreq(static_cast<int>(devconfig.freq[0]));
                    config_device.freq2 =
                        getMultiFreq(static_cast<int>(devconfig.freq[1]));
                    config_device.freq3 =
                        getMultiFreq(static_cast<int>(devconfig.freq[2]));
                    config_device.freq4 = getMultiFreq(static_cast<int>(devconfig.freq[3]));
                    config_device.freq = static_cast<int>(devconfig.modFreq);

                    config_device.int1 =
                        static_cast<int>(devconfig.integrationTimes[0]);
                    config_device.int2 =
                        static_cast<int>(devconfig.integrationTimes[1]);
                    config_device.int3 =
                        static_cast<int>(devconfig.integrationTimes[2]);
                    config_device.int4 = static_cast<int>(devconfig.integrationTimes[3]);
                    config_device.intgs =
                        static_cast<int>(devconfig.integrationTimeGs);

                    config_device.imgType = static_cast<int>(4);
                    config_device.HDR = static_cast<int>(devconfig.hdrMode);
                    config_device.maxfps = static_cast<int>(devconfig.maxfps);
                    config_device.minLSB = static_cast<int>(devconfig.miniAmp);
                    std::cout << "******************* GET CONFIG SUCCESS "
                                 "*******************"
                              << std::endl;
                    std::cout
                        << "version: " << std::to_string(devconfig.version)
                        << std::endl;
                    // std::cout << "freq: " <<
                    // ModulationFreqStr[devconfig.freq[0]] << " " <<
                    // ModulationFreqStr[devconfig.freq[1]] << " " <<
                    // ModulationFreqStr[devconfig.freq[2]] << " " <<
                    // ModulationFreqStr[devconfig.freq[3]] << std::endl;
                    // std::cout << "ImageType: " <<
                    // ImageTypeStr[devconfig.imgType] << std::endl; std::cout
                    // << "ModulationFreq: " <<
                    // ModulationFreqStr[devconfig.modFreq] << std::endl;
                    // std::cout << "HDRMode: " <<
                    // HDRModeStr[devconfig.hdrMode] << std::endl;

                    // std::cout << "integrationTimes: " <<
                    // std::to_string(devconfig.integrationTimes[0]) << " "
                    //           <<
                    //           std::to_string(devconfig.integrationTimes[1])
                    //           << " "
                    //           <<
                    //           std::to_string(devconfig.integrationTimes[2])
                    //           << " "
                    //           <<
                    //           std::to_string(devconfig.integrationTimes[3])
                    //           << std::endl;
                    // std::cout << "integrationTimeGs: " <<
                    // std::to_string(devconfig.integrationTimeGs) <<
                    // std::endl; std::cout << "miniAmp: " <<
                    // std::to_string(devconfig.miniAmp) << std::endl;
                    // std::cout << "setmaxfps: " <<
                    // std::to_string(devconfig.setmaxfps) << std::endl;

                    std::cout << "freq: " << ModulationFreqStr[config_device.freq1] << " "
                              << ModulationFreqStr[config_device.freq2] << " "
                              << ModulationFreqStr[config_device.freq3] << " "
                              << ModulationFreqStr[config_device.freq4] << std::endl;
                    std::cout << "ImageType: " << config_device.imgType
                              << std::endl;
                    std::cout << "ModulationFreq: " << config_device.freq
                              << std::endl;
                    std::cout << "HDRMode: " << config_device.HDR << std::endl;
                    std::cout << "integrationTimes: " << config_device.int1 << " "
                              << config_device.int2 << " " << config_device.int3 << " "
                              << config_device.int4 << std::endl;
                    std::cout << "integrationTimeGs : " << config_device.intgs
                              << std::endl;
                    std::cout << "miniAmp: " << config_device.minLSB
                              << std::endl;
                    std::cout << "setmaxfps: " << config_device.maxfps
                              << std::endl;

                    std::cout << "isFilterOn: "
                              << std::to_string(devconfig.isFilterOn)
                              << std::endl;
                    std::cout << "roi: " << std::to_string(devconfig.roi[0])
                              << " " << std::to_string(devconfig.roi[1]) << " "
                              << std::to_string(devconfig.roi[2]) << " "
                              << std::to_string(devconfig.roi[3]) << std::endl;
                    std::cout << "maxfps: " << std::to_string(devconfig.maxfps)
                              << std::endl;
                    std::cout << "bCompensateOn: "
                              << std::to_string(devconfig.bCompensateOn)
                              << std::endl;
                    std::cout
                        << "bBinningH: " << std::to_string(devconfig.bBinningH)
                        << std::endl;
                    std::cout
                        << "bBinningV: " << std::to_string(devconfig.bBinningV)
                        << std::endl;
                    std::cout << "freqChannel: "
                              << std::to_string(devconfig.freqChannel)
                              << std::endl;

                    std::cout << "endianType: "
                              << std::to_string(devconfig.endianType)
                              << std::endl;

                    std::cout << "bcut_filteron: "
                              << std::to_string(devconfig.bcut_filteron)
                              << std::endl;
                    std::cout << "cut_intgrttime0: "
                              << std::to_string(devconfig.cut_intgrttime0)
                              << std::endl;
                    std::cout << "cut_distance0: "
                              << std::to_string(devconfig.cut_distance0)
                              << std::endl;
                    std::cout << "cut_intgrttime1: "
                              << std::to_string(devconfig.cut_intgrttime1)
                              << std::endl;
                    std::cout << "cut_distance1: "
                              << std::to_string(devconfig.cut_distance1)
                              << std::endl;
                    std::cout << "*********************************************"
                                 "***********"
                              << std::endl;
                    // if (!is_connected) {
                    //     is_connected = true;
                    //     return;
                    // }
                    // update_config_from_dev(config);
                    // server.updateConfig(config);

                    // is_connected += 1;
                }
            }
            xtsdk->setCutCorner(para_set.lidar_setting_.cut_corner);
            // xtsdk->setTransMirror(1, 1);

            xtsdk->getImuExtParamters(e_imu_lidar, 1);
            is_connected += 1;
            is_connected = is_connected > 10 ? 10 : is_connected;
        }

        // if (!para_set.lidar_setting_.usb_com && xtsdk->getStateStr() ==
        // "PortOpening-Disconnected") {
        //     xtsdk->stop();
        //     xtsdk->setConnectIpaddress(para_set.lidar_setting_.connect_address);
        //     xtsdk->start((ImageType)para_set.lidar_setting_.imgType);
        //     // ROS_INFO("usb com to connect_address %s",
        //     // para_set.lidar_setting_.connect_address.c_str());
        // }
    }
    else if (event->eventstr == "devState")
    {

        // if (!para_set.lidar_setting_.usb_com && xtsdk->getStateStr() ==
        // "PortOpening-Disconnected") {
        //     xtsdk->stop();
        //     xtsdk->setConnectIpaddress(para_set.lidar_setting_.connect_address);
        //     xtsdk->start((ImageType)para_set.lidar_setting_.imgType);
        //     // ROS_INFO("usb com to connect_address %s",
        //     // para_set.lidar_setting_.connect_address.c_str());
        // }
        // std::cout << "devstate= " + xtsdk->getStateStr() << std::endl;
        // XTAPPLOG("devstate= " + xtsdk->getStateStr());
    }

    else
    {
        if (event->cmdid == 252)
        {
            if (is_connected == 0)
            {
                return;
            }
            // if (imucount > 10)
            // {
            // imucount = 0;
            FrameOutImu_t *pimu = (FrameOutImu_t *)(event->data.data());
            float imutemp = *(float *)&pimu->data[9];
            float imuaccx = *(float *)&pimu->data[0];
            float imuaccy = *(float *)&pimu->data[1];
            float imuaccz = *(float *)&pimu->data[2];
            float imugx = *(float *)&pimu->data[3];
            float imugy = *(float *)&pimu->data[4];
            float imugz = *(float *)&pimu->data[5];
            uint64_t imutimestamp = *(uint64_t *)&pimu->data[10];
            uint32_t times = 0;
            uint32_t timems = 0;
            if (imutimestamp > 100000000000) // ptp同步时
            {
                times = ((imutimestamp / 1000 - 37) % 60);
                timems = imutimestamp % 1000;
            }
            else
            {
                times = imutimestamp / 1000;
                timems = imutimestamp % 1000;
            }
            imucount++;
            // }

            if (imucount > 10)
            {
                std::cout << "linear_acceleration: " << imuaccx << " " << imuaccy << " " << imuaccz << std::endl;

                // std::cout << times << std::endl;
                // std::cout << timems << std::endl;
                imucount = 0;
            }
        }
    }
}

bool readIniFile(const std::string &filename, para_ros &para_,
                 xtsdk_ros::xtsdk_ros1Config &config,
                 const xtsdk_ros::xtsdk_ros1Config &config_default)
{
    try
    {
        boost::property_tree::ptree pt;
        boost::property_tree::ini_parser::read_ini(filename, pt);

        // settings

        config.freq = pt.get<int>("Setting.freq", config_default.freq);
        config.freq1 = pt.get<int>("Setting.freq1", config_default.freq1);
        config.freq2 = pt.get<int>("Setting.freq2", config_default.freq2);
        config.freq3 = pt.get<int>("Setting.freq3", config_default.freq3);
        config.freq4 = pt.get<int>("Setting.freq4", config_default.freq4);
        config.HDR = pt.get<int>("Setting.HDR", config_default.HDR);
        config.imgType = pt.get<int>("Setting.imgType", config_default.imgType);
        config.cloud_coord =
            pt.get<int>("Setting.cloud_coord", config_default.cloud_coord);

        config.int1 = pt.get<int>("Setting.int1", config_default.int1);
        config.int2 = pt.get<int>("Setting.int2", config_default.int2);
        config.int3 = pt.get<int>("Setting.int3", config_default.int3);
        config.int4 = pt.get<int>("Setting.int4", config_default.int4);
        config.intgs = pt.get<int>("Setting.intgs", config_default.intgs);
        config.minLSB = pt.get<int>("Setting.minLSB", config_default.minLSB);
        config.cut_corner = pt.get<int>("Setting.cut_corner", config_default.cut_corner);

        config.start_stream =
            pt.get<bool>("Setting.start_stream", config_default.start_stream);
        config.gray_on = pt.get<bool>("Setting.gray_on", config_default.gray_on);
        config.is_use_devconfig =
            pt.get<bool>("Setting.is_use_devconfig", config_default.is_use_devconfig);

        config.maxfps = pt.get<int>("Setting.maxfps", config_default.maxfps);
        // config.usb_com =
        //     pt.get<bool>("Setting.usb_com", config_default.usb_com);
        // config.usb_com_name = pt.get<std::string>("Setting.usb_com_name",
        //                                           config_default.usb_com_name);

        config.hmirror = pt.get<bool>("Setting.hmirror", config_default.hmirror);
        config.vmirror = pt.get<bool>("Setting.vmirror", config_default.vmirror);
        config.binningV = pt.get<bool>("Setting.binningV", config_default.binningV);

        // filters
        para_.lidar_filter_.medianSize = pt.get<int>("Filters.medianSize", 5);
        para_.lidar_filter_.kalmanEnable = pt.get<int>("Filters.kalmanEnable", 1);
        para_.lidar_filter_.kalmanFactor = pt.get<float>("Filters.kalmanFactor", 0.4);
        para_.lidar_filter_.kalmanThreshold = pt.get<int>("Filters.kalmanThreshold", 400);
        para_.lidar_filter_.edgeEnable = pt.get<bool>("Filters.edgeEnable", true);
        para_.lidar_filter_.edgeThreshold = pt.get<int>("Filters.edgeThreshold", 200);
        para_.lidar_filter_.dustEnable = pt.get<bool>("Filters.dustEnable", true);
        para_.lidar_filter_.dustThreshold = pt.get<int>("Filters.dustThreshold", 2002);
        para_.lidar_filter_.dustFrames = pt.get<int>("Filters.dustFrames", 2);

        para_.lidar_filter_.postprocessEnable = pt.get<bool>("Filters.postprocessEnable", true);
        para_.lidar_filter_.dynamicsEnabled = pt.get<bool>("Filters.dynamicsEnabled", true);
        para_.lidar_filter_.dynamicsWinsize = pt.get<int>("Filters.dynamicsWinsize", 9);
        para_.lidar_filter_.dynamicsMotionsize = pt.get<int>("Filters.dynamicsMotionsize", 5);
        para_.lidar_filter_.reflectiveEnable = pt.get<bool>("Filters.reflectiveEnable", true);
        para_.lidar_filter_.ref_th_min = pt.get<float>("Filters.ref_th_min", 0.5);
        para_.lidar_filter_.ref_th_max = pt.get<float>("Filters.ref_th_max", 2.0);
        para_.lidar_filter_.postprocessThreshold =
            pt.get<float>("Filters.postprocessThreshold", 5.0);

        // ros

        para_.lidar_ros_.frame_id =
            pt.get<std::string>("Setting.frame_id", config_default.frame_id);
        para_.lidar_ros_.topic_name = pt.get<std::string>(
            "Setting.topic_name", config_default.topic_name);

        // para_.lidar_setting_.usb_com = pt.get<bool>("Setting.usb_com",
        // config_default.usb_com);
        std::cout << "Setting.usb_com: " << para_.lidar_setting_.usb_com
                  << std::endl;
        // para_.lidar_setting_.usb_com_name =
        // pt.get<std::string>("Setting.usb_com_name",
        // config_default.usb_com_name);

        //// sync communication status
        config.usb_com = para_set.lidar_setting_.usb_com;
        config.connect_address = para_set.lidar_setting_.connect_address;
        config.usb_com_name = para_set.lidar_setting_.usb_com_name;

        return true;
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Error reading ini file: %s", e.what());

        config = config_default;
        para_.lidar_setting_.hmirror = false;
        para_.lidar_setting_.vmirror = false;

        // filters
        para_.lidar_filter_.medianSize = 5;
        para_.lidar_filter_.kalmanEnable = 1;
        para_.lidar_filter_.kalmanFactor = 0.4;
        para_.lidar_filter_.kalmanThreshold = 400;
        para_.lidar_filter_.edgeEnable = true;
        para_.lidar_filter_.edgeThreshold = 200;
        para_.lidar_filter_.dustEnable = true;
        para_.lidar_filter_.dustThreshold = 2002;
        para_.lidar_filter_.dustFrames = 2;
        para_.lidar_filter_.postprocessEnable = true;
        para_.lidar_filter_.dynamicsEnabled = true;
        para_.lidar_filter_.dynamicsWinsize = 9;
        para_.lidar_filter_.dynamicsMotionsize = 5;
        para_.lidar_filter_.reflectiveEnable = true;
        para_.lidar_filter_.ref_th_min = 0.5;
        para_.lidar_filter_.ref_th_max = 2.0;
        para_.lidar_filter_.postprocessThreshold = 5.0;

        // ros

        para_.lidar_ros_.frame_id = config_default.frame_id;
        para_.lidar_ros_.topic_name = config_default.topic_name;
        para_.lidar_setting_.connect_address = config_default.connect_address;
        para_.lidar_setting_.usb_com = config_default.usb_com;
        para_.lidar_setting_.usb_com_name = config_default.usb_com_name;
        para_.lidar_setting_.gray_on = config_default.gray_on;
        para_.lidar_setting_.is_use_devconfig = config_default.is_use_devconfig;

        //// sync communication status
        config.usb_com = para_set.lidar_setting_.usb_com;
        config.connect_address = para_set.lidar_setting_.connect_address;
        config.usb_com_name = para_set.lidar_setting_.usb_com_name;
        return false;
    }
}
void setParaToinit()
{
    para_set.lidar_setting_.freq1 = -1;
    para_set.lidar_setting_.freq2 = -1;
    para_set.lidar_setting_.freq3 = -1;
    para_set.lidar_setting_.freq4 = -1;
    para_set.lidar_setting_.freq = -1;

    para_set.lidar_setting_.int1 = -1;
    para_set.lidar_setting_.int2 = -1;
    para_set.lidar_setting_.int3 = -1;
    para_set.lidar_setting_.int4 = -1;
    para_set.lidar_setting_.intgs = -1;

    para_set.lidar_setting_.imgType = -1;
    para_set.lidar_setting_.HDR = -1;
    para_set.lidar_setting_.maxfps = -1;
    para_set.lidar_setting_.minLSB = -1;
}
void signalHandler(int signum)
{
    ROS_INFO("Received signal: %d", signum);
    keepRunning = false;
    ros::shutdown(); // 确保调用
}
//==========================================================================
int main(int argc, char **argv)
{
    xtsdk = new XtSdk();
    xtsdk->setCallback(eventCallback, imgCallback);

    ROS_INFO("Started");

    ros::init(argc, argv, "xtsdk_ros");

    // 动态参数
    boost::recursive_mutex config_mutex;
    dynamic_reconfigure::Server<xtsdk_ros::xtsdk_ros1Config> server(
        config_mutex);
    dynamic_reconfigure::Server<xtsdk_ros::xtsdk_ros1Config>::CallbackType f;
    f = boost::bind(&updateConfig, _1, _2);

    server.setCallback(f);

    xtsdk_ros::xtsdk_ros1Config config_default;
    std::string xt_pkg = "xtsdk_ros";
    std::string xt_pkg_cfg_path =
        ros::package::getPath(xt_pkg) + "/cfg/xintan.xtcfg";
    server.getConfigDefault(config_default);
    // std::cout << "default configuration: " << config_default.frame_id <<
    // std::endl;
    std::string connect_add = config_default.usb_com
                                  ? config_default.usb_com_name
                                  : config_default.connect_address;
    initialise(config_default.topic_name);
    xtsdk->setConnectIpaddress(connect_add);
    xtsdk->startup();
    para_set.lidar_setting_.connect_address = config_default.connect_address;
    para_set.lidar_setting_.usb_com = config_default.usb_com;
    para_set.lidar_setting_.usb_com_name = config_default.usb_com_name;
    para_set.lidar_setting_.is_use_devconfig = config_default.is_use_devconfig;

    while (is_connected == 0 && keepRunning)
    {
        ros::spinOnce();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    is_connected_once = is_connected;
    setParaToinit();
    if (readIniFile(xt_pkg_cfg_path, para_set, config, config_default))
    {

        // boost::recursive_mutex::scoped_lock lock(config_mutex);
        ROS_INFO("INI READING SUCCESSFUL SETTING");
        if (get_config_success && para_set.lidar_setting_.is_use_devconfig)
        {
            ROS_INFO("UPDATE DEVICE CONFIG");
            update_config_from_dev(config);
        }

        server.updateConfig(config);
    }
    else
    {
        if (get_config_success && para_set.lidar_setting_.is_use_devconfig)
        {
            ROS_INFO("UPDATE DEVICE CONFIG");
            update_config_from_dev(config);
        }
        ROS_ERROR("Failed to read initial configuration from ini file. Using "
                  "default values.");
        server.updateConfig(config);
    }
    server.setCallback(f);

    std::string connect_addr = para_set.lidar_setting_.usb_com
                                   ? para_set.lidar_setting_.usb_com_name
                                   : para_set.lidar_setting_.connect_address;
    xtsdk->setConnectIpaddress(connect_addr);
    xtsdk->start((ImageType)para_set.lidar_setting_.imgType);
    // xtsdk->setConnectIpaddress(para_set.lidar_setting_.connect_address);
    // xtsdk->setConnectSerialportName("COM3");//ubuntu 下设备名是
    // "/dev/ttyACM0"

    // xtsdk->setConnectSerialportName("/dev/ttyACM0"); // linux

    if (para_set.lidar_filter_.reflectiveEnable)
    {
        xtsdk->setSdkReflectiveFilter(para_set.lidar_filter_.ref_th_min,
                                      para_set.lidar_filter_.ref_th_max);
    }
    if (para_set.lidar_filter_.edgeEnable)
    {
        xtsdk->setSdkEdgeFilter(para_set.lidar_filter_.edgeThreshold);
        // xtsdk->setSdkEdgeFilter(200);
    }
    if (para_set.lidar_filter_.kalmanEnable)
    {
        xtsdk->setSdkKalmanFilter(para_set.lidar_filter_.kalmanFactor * 1000, para_set.lidar_filter_.kalmanThreshold, 2000); // sdk中开启卡尔曼滤波
        // xtsdk->setSdkKalmanFilter(300, 300, 2000);
    }
    if (para_set.lidar_filter_.medianSize > 0)
    {
        xtsdk->setSdkMedianFilter(para_set.lidar_filter_.medianSize); // sdk中执行3*3的中值滤波
        // xtsdk->setSdkMedianFilter(3);
    }
    if (para_set.lidar_filter_.dustEnable)
    {
        std::cout << para_set.lidar_filter_.dustThreshold << std::endl;
        xtsdk->setSdkDustFilter(para_set.lidar_filter_.dustThreshold, para_set.lidar_filter_.dustFrames);

        // xtsdk->setSdkDustFilter(2002, 6);
    }
    if (para_set.lidar_filter_.postprocessEnable)
    {
        xtsdk->setPostProcess(para_set.lidar_filter_.postprocessThreshold,
                              static_cast<uint8_t>(para_set.lidar_filter_.dynamicsEnabled),
                              para_set.lidar_filter_.dynamicsWinsize,
                              para_set.lidar_filter_.dynamicsMotionsize);
    }
    // 设置接收图流的UDP端口，要在第一次调用xtsdk->startup()之前使用, 网络中只有一台设备的不需要设置
    // xtsdk->setUdpDestIp("", 7601);

    std::signal(SIGINT, signalHandler);
    std::thread worker([&server, &f]()
                       {
        while (keepRunning) {
            if (is_connected_once < is_connected) {
                ROS_INFO("DEVICE RECONNECTED");
                setParaToinit();
                xtsdk->stop();
                is_connected_once = is_connected;
                if (get_config_success) {
                    // config.usb_com = para_set.lidar_setting_.usb_com;
                    ROS_INFO("UPDATE DEVICE CONFIG");
                    update_config_from_dev(config);
                }
                config.usb_com = para_set.lidar_setting_.usb_com;
                config.connect_address =
                    para_set.lidar_setting_.connect_address;
                config.usb_com_name = para_set.lidar_setting_.usb_com_name;
                server.updateConfig(config);
                ROS_INFO("UPDATE  RECONFIG");
                server.setCallback(f);
                std::this_thread::sleep_for(std::chrono::duration<double>(0.5));

                std::string connect_addr =
                    para_set.lidar_setting_.usb_com
                        ? para_set.lidar_setting_.usb_com_name
                        : para_set.lidar_setting_.connect_address;
                xtsdk->setConnectIpaddress(connect_addr);
                xtsdk->start((ImageType)para_set.lidar_setting_.imgType);
            }

            // ROS_INFO ("KEEP RUNNING");
            std::this_thread::sleep_for(std::chrono::duration<double>(0.1));
            ros::spinOnce();
        } });
    ros::spin();

    worker.join();

    xtsdk->stop();
    xtsdk->setCallback();
    xtsdk->shutdown();
    std::cout << "Terminated " << std::endl;
}
