
#ifndef __PARA_ROS_H__
#define __PARA_ROS_H__
#include <string>

struct lidar_setting
{
    int imgType;
    int HDR;
    int minLSB;
    int freq;
    int cloud_coord;
    int int1, int2, int3, int4, intgs;
    int freq1, freq2, freq3, freq4;
    int cut_corner;
    int maxfps;

    bool start_stream;
    bool hmirror;
    bool vmirror;

    bool usb_com;
    bool gray_on;
    bool is_use_devconfig;
    std::string usb_com_name;
    std::string connect_address;
};
struct lidar_filter
{
    int medianSize;
    bool kalmanEnable;
    float kalmanFactor;
    int kalmanThreshold;
    bool edgeEnable;
    int edgeThreshold;
    bool dustEnable;
    int dustThreshold;
    int dustFrames;
    bool postprocessEnable;
    bool dynamicsEnabled;
    int dynamicsWinsize;
    int dynamicsMotionsize;
    float postprocessThreshold;
    bool reflectiveEnable;
    float ref_th_min;
    float ref_th_max;
};

struct lidar_ros
{
    std::string topic_name;
    std::string frame_id;
};

struct para_ros
{
    lidar_setting lidar_setting_;
    lidar_filter lidar_filter_;
    lidar_ros lidar_ros_;
};

#endif