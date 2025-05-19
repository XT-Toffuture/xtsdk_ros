#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/parameter_event.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <yaml-cpp/yaml.h>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <csignal>
#include "xtsdk.h"
#include "para_ros.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

using namespace XinTan;

rclcpp::Node::SharedPtr g_node = nullptr;
uint8_t check_count = 0;
XtSdk *xtsdk;
uint8_t is_connected = 0;
bool get_config_success = false;
bool update_once = false;
double fwversionf = 0.0;
// 动态参数结构

static std::string connect_address;

// topic 对象
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointCloud2Publisher;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr distanceImagePublisher;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr amplitudeImagePublisher;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr grayImagePublisher;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr eventPublisher;
rclcpp::Publisher<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_pub_;

static para_ros para_set;
static para_ros config_device;

void updateParaFromDev()
{
    std::vector<rclcpp::Parameter> parameters;

    if (get_config_success && para_set.lidar_setting_.is_use_devconfig)
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "UPDATE PARA FROM DEVICE");
        parameters.push_back(rclcpp::Parameter("int1", config_device.lidar_setting_.int1));
        parameters.push_back(rclcpp::Parameter("int2", config_device.lidar_setting_.int2));
        parameters.push_back(rclcpp::Parameter("int3", config_device.lidar_setting_.int3));
        parameters.push_back(rclcpp::Parameter("int4", config_device.lidar_setting_.int4));
        parameters.push_back(rclcpp::Parameter("intgs", config_device.lidar_setting_.intgs));
        parameters.push_back(rclcpp::Parameter("freq1", config_device.lidar_setting_.freq1));
        parameters.push_back(rclcpp::Parameter("freq2", config_device.lidar_setting_.freq2));
        parameters.push_back(rclcpp::Parameter("freq3", config_device.lidar_setting_.freq3));
        parameters.push_back(rclcpp::Parameter("freq4", config_device.lidar_setting_.freq4));

        parameters.push_back(rclcpp::Parameter("imgType", config_device.lidar_setting_.imgType));
        parameters.push_back(rclcpp::Parameter("HDR", config_device.lidar_setting_.HDR));
        parameters.push_back(rclcpp::Parameter("maxfps", config_device.lidar_setting_.maxfps));
        parameters.push_back(rclcpp::Parameter("minLSB", config_device.lidar_setting_.minLSB));
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "UPDATE PARA FROM DEFAULT");
        parameters.push_back(rclcpp::Parameter("int1", para_set.lidar_setting_.int1));
        parameters.push_back(rclcpp::Parameter("int2", para_set.lidar_setting_.int2));
        parameters.push_back(rclcpp::Parameter("int3", para_set.lidar_setting_.int3));
        parameters.push_back(rclcpp::Parameter("int3", para_set.lidar_setting_.int4));
        parameters.push_back(rclcpp::Parameter("intgs", para_set.lidar_setting_.intgs));
        parameters.push_back(rclcpp::Parameter("freq1", para_set.lidar_setting_.freq1));
        parameters.push_back(rclcpp::Parameter("freq2", para_set.lidar_setting_.freq2));
        parameters.push_back(rclcpp::Parameter("freq3", para_set.lidar_setting_.freq3));
        parameters.push_back(rclcpp::Parameter("freq3", para_set.lidar_setting_.freq4));

        parameters.push_back(rclcpp::Parameter("freq", para_set.lidar_setting_.freq));
        parameters.push_back(rclcpp::Parameter("imgType", para_set.lidar_setting_.imgType));
        parameters.push_back(rclcpp::Parameter("HDR", para_set.lidar_setting_.HDR));
        parameters.push_back(rclcpp::Parameter("maxfps", para_set.lidar_setting_.maxfps));
        parameters.push_back(rclcpp::Parameter("minLSB", para_set.lidar_setting_.minLSB));
    }

    // 更新多个参数
    g_node->set_parameters(parameters);
}
void setAmpType_reflect(std::string devsn)
{

    uint16_t ampnormalizedv = 20;
    XByteArray data = {
        static_cast<uint8_t>(20),
        static_cast<uint8_t>(ampnormalizedv >> 8), static_cast<uint8_t>(ampnormalizedv & 0x00ff)};
    XByteArray respdata;
    xtsdk->customCmd(202, data, respdata);
}
void print_curr_para()
{

    std::cout << "usb_com: " << para_set.lidar_setting_.usb_com << std::endl;
    std::cout << "usb_com_name: " << para_set.lidar_setting_.usb_com_name << std::endl;
    std::cout << "connect_address: " << para_set.lidar_setting_.connect_address << std::endl;
    std::cout << "cut_corner: " << para_set.lidar_setting_.cut_corner << std::endl;
    std::cout << "freq: " << para_set.lidar_setting_.freq << std::endl;
    std::cout << "freq1: " << para_set.lidar_setting_.freq1 << std::endl;
    std::cout << "freq2: " << para_set.lidar_setting_.freq2 << std::endl;
    std::cout << "freq3: " << para_set.lidar_setting_.freq3 << std::endl;
    std::cout << "freq4: " << para_set.lidar_setting_.freq4 << std::endl;
    std::cout << "HDR: " << para_set.lidar_setting_.HDR << std::endl;
    std::cout << "hmirror: " << para_set.lidar_setting_.hmirror << std::endl;
    std::cout << "imgType: " << para_set.lidar_setting_.imgType << std::endl;
    std::cout << "cloud_coord: " << para_set.lidar_setting_.cloud_coord << std::endl;
    std::cout << "int1: " << para_set.lidar_setting_.int1 << std::endl;
    std::cout << "int2: " << para_set.lidar_setting_.int2 << std::endl;
    std::cout << "int3: " << para_set.lidar_setting_.int3 << std::endl;
    std::cout << "int4: " << para_set.lidar_setting_.int4 << std::endl;
    std::cout << "intgs: " << para_set.lidar_setting_.intgs << std::endl;
    std::cout << "maxfps: " << para_set.lidar_setting_.maxfps << std::endl;
    std::cout << "minLSB: " << para_set.lidar_setting_.minLSB << std::endl;
    std::cout << "start_stream: " << para_set.lidar_setting_.start_stream << std::endl;
    std::cout << "vmirror: " << para_set.lidar_setting_.vmirror << std::endl;
    std::cout << "gray_on: " << para_set.lidar_setting_.gray_on << std::endl;

    std::cout << "dustEnable: " << para_set.lidar_filter_.dustEnable << std::endl;
    std::cout << "dustFrames: " << para_set.lidar_filter_.dustFrames << std::endl;
    std::cout << "dustThreshold: " << para_set.lidar_filter_.dustThreshold << std::endl;
    std::cout << "edgeEnable: " << para_set.lidar_filter_.edgeEnable << std::endl;
    std::cout << "edgeThreshold: " << para_set.lidar_filter_.edgeThreshold << std::endl;
    std::cout << "kalmanEnable: " << para_set.lidar_filter_.kalmanEnable << std::endl;
    std::cout << "kalmanFactor: " << para_set.lidar_filter_.kalmanFactor << std::endl;
    std::cout << "kalmanThreshold: " << para_set.lidar_filter_.kalmanThreshold << std::endl;
    std::cout << "medianSize: " << para_set.lidar_filter_.medianSize << std::endl;

    std::cout << "frame_id: " << para_set.lidar_ros_.frame_id << std::endl;
    std::cout << "topic_name: " << para_set.lidar_ros_.topic_name << std::endl;
}

class XtsdkNode : public rclcpp::Node
{
public:
    XtsdkNode()
        : Node("xtsdk_node")
    {
        // 动态参数改变回调
        callback_handle_ = this->add_on_set_parameters_callback(std::bind(&XtsdkNode::SetParametersCallback, this, std::placeholders::_1, &para_set));
        // std::string packagedirectory = ament_index_cpp::get_package_share_directory("xtsdk_ros");
        std::string package_directory = std::string(PACKAGE_DIR);
        std::string param_config_yaml = package_directory + "/cfg/xtsdk_ros2.yaml";
        std::string param_config_ini = package_directory + "/cfg/xintan.xtcfg";
        set_init_parameter(param_config_yaml, param_config_ini);

        rcl_interfaces::msg::ParameterDescriptor descriptor_num;
        // rcl_interfaces::msg::ParameterDescriptor descriptor_enum;
        descriptor_num.description = "";
        descriptor_num.name = "name";
        descriptor_num.integer_range.resize(1);

        for (auto enum_pair : enum_map)
        {
            double min_value = std::numeric_limits<double>::max();
            double max_value = std::numeric_limits<double>::lowest();

            for (const auto &pair : enum_pair.second)
            {
                if (pair.second < min_value)
                {
                    min_value = pair.second;
                }
                if (pair.second > max_value)
                {
                    max_value = pair.second;
                }
            }

            rcl_interfaces::msg::ParameterDescriptor descriptor_enum;
            rcl_interfaces::msg::IntegerRange int_range;

            int_range.from_value = min_value;
            int_range.to_value = max_value;
            int_range.step = 1;
            descriptor_enum.integer_range.push_back(int_range);

            // Use the default value from enum_map to declare the parameter
            this->declare_parameter<int>(enum_pair.first, enum_pair.second.at("default"), descriptor_enum);
        }

        for (auto num_pair : numb_map)
        {

            descriptor_num.integer_range[0].from_value = num_pair.second["min"];
            descriptor_num.integer_range[0].to_value = num_pair.second["max"];
            descriptor_num.integer_range[0].step = 1;

            this->declare_parameter<int32_t>(num_pair.first, num_pair.second["default"], descriptor_num);
        }

        for (auto bool_pair : bool_map)
        {
            this->declare_parameter<bool>(bool_pair.first, bool_pair.second);
        }

        for (auto gen_pair : general_map)
        {
            this->declare_parameter<std::string>(gen_pair.first, gen_pair.second);
        }

        // 构建配置文件路径

        // topic 对象实例化
        pointCloud2Publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("xtsdk_ros/xtsdk_node/" + para_set.lidar_ros_.topic_name, 10);
        distanceImagePublisher = this->create_publisher<sensor_msgs::msg::Image>("xtsdk_ros/xtsdk_node/distance_image_raw", 10);
        amplitudeImagePublisher = this->create_publisher<sensor_msgs::msg::Image>("xtsdk_ros/xtsdk_node/amplitude_image_raw", 10);
        grayImagePublisher = this->create_publisher<sensor_msgs::msg::Image>("xtsdk_ros/xtsdk_node/gray_image_raw", 10);
        eventPublisher = this->create_publisher<std_msgs::msg::String>("xtsdk_ros/xtsdk_node/event", 10);
        parameter_event_pub_ = this->create_publisher<rcl_interfaces::msg::ParameterEvent>("/parameter_events", 10);
    }

private:
    // 动态参数回调函数
    void set_init_parameter(const std::string &yaml_path, const std::string &ini_path)
    {
        bool read_ini_success = false;
        boost::property_tree::ptree pt;
        try
        {
            boost::property_tree::ini_parser::read_ini(ini_path, pt);
            read_ini_success = true;
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load INI file: %s and load default parameter", e.what());
            read_ini_success = false;
        }

        std::cout << "Parameter config file: " << yaml_path << std::endl;
        YAML::Node config = YAML::LoadFile(yaml_path);
        if (config.IsNull())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load YAML file: %s", yaml_path.c_str());
            return;
        }

        if (config["enum_para"])
        {
            YAML::Node config1 = config["enum_para"];
            for (auto iter = config1.begin(); iter != config1.end(); iter++)
            {
                std::map<std::string, int> tmp;
                auto node = iter->second;
                for (auto iter1 = node.begin(); iter1 != node.end(); iter1++)
                {
                    tmp[iter1->first.as<std::string>()] = iter1->second.as<int>();
                }
                enum_map[iter->first.as<std::string>()] = tmp;
            }
        }
        if (config["num_para"])
        {
            YAML::Node config1 = config["num_para"];
            for (auto iter = config1.begin(); iter != config1.end(); iter++)
            {
                std::map<std::string, int> tmp;
                auto node = iter->second;
                for (auto iter1 = node.begin(); iter1 != node.end(); iter1++)
                {
                    tmp[iter1->first.as<std::string>()] = iter1->second.as<int>();
                }
                numb_map[iter->first.as<std::string>()] = tmp;
            }
        }

        if (config["bool_para"])
        {
            YAML::Node config1 = config["bool_para"];
            for (auto iter = config1.begin(); iter != config1.end(); iter++)
            {
                bool_map[iter->first.as<std::string>()] = iter->second.as<bool>();
            }
        }
        if (config["general_para"])
        {
            YAML::Node config1 = config["general_para"];
            for (auto iter = config1.begin(); iter != config1.end(); iter++)
            {
                general_map[iter->first.as<std::string>()] = iter->second.as<std::string>();
            }
        }
        if (config["default"])
        {
            YAML::Node config1 = config["default"];
            for (auto iter = config1.begin(); iter != config1.end(); iter++)
            {
                auto key_tmp = iter->first.as<std::string>();
                boost::property_tree::path path_tmp("Setting." + key_tmp);
                if (enum_map.count(key_tmp) > 0)
                {
                    enum_map[key_tmp]["default"] = read_ini_success ? pt.get<int>(path_tmp, iter->second.as<int>()) : iter->second.as<int>();
                }
                else if (numb_map.count(key_tmp) > 0)
                {
                    numb_map[key_tmp]["default"] = read_ini_success ? pt.get<int>(path_tmp, iter->second.as<int>()) : iter->second.as<int>();
                }
                else if (bool_map.count(key_tmp) > 0)
                {
                    bool_map[key_tmp] = read_ini_success ? pt.get<bool>(path_tmp, iter->second.as<bool>()) : iter->second.as<bool>();
                }
                else if (general_map.count(key_tmp) > 0)
                {
                    general_map[key_tmp] = read_ini_success ? pt.get<std::string>(path_tmp, iter->second.as<std::string>()) : iter->second.as<std::string>();
                }
            }
        }

        readIniFile(read_ini_success, pt, para_set);
    }

    void readIniFile(bool read_ini_success,
                     boost::property_tree::ptree pt,
                     para_ros &para_)
    {

        // settings
        if (read_ini_success)
        {
            para_.lidar_setting_.freq = pt.get<int>("Setting.freq", enum_map["freq"]["default"]);
            para_.lidar_setting_.freq1 = pt.get<int>("Setting.freq1", enum_map["freq1"]["default"]);
            para_.lidar_setting_.freq2 = pt.get<int>("Setting.freq2", enum_map["freq2"]["default"]);
            para_.lidar_setting_.freq3 = pt.get<int>("Setting.freq3", enum_map["freq3"]["default"]);
            para_.lidar_setting_.freq4 = pt.get<int>("Setting.freq3", enum_map["freq4"]["default"]);
            para_.lidar_setting_.HDR = pt.get<int>("Setting.HDR", enum_map["HDR"]["default"]);
            para_.lidar_setting_.imgType = pt.get<int>("Setting.imgType", enum_map["imgType"]["default"]);
            para_.lidar_setting_.cloud_coord = pt.get<int>("Setting.cloud_coord", enum_map["cloud_coord"]["default"]);

            para_.lidar_setting_.int1 = pt.get<int>("Setting.int1", numb_map["int1"]["default"]);
            para_.lidar_setting_.int2 = pt.get<int>("Setting.int2", numb_map["int2"]["default"]);
            para_.lidar_setting_.int3 = pt.get<int>("Setting.int3", numb_map["int3"]["default"]);
            para_.lidar_setting_.int4 = pt.get<int>("Setting.int3", numb_map["int4"]["default"]);
            para_.lidar_setting_.intgs = pt.get<int>("Setting.intgs", numb_map["intgs"]["default"]);
            para_.lidar_setting_.minLSB = pt.get<int>("Setting.minLSB", numb_map["minLSB"]["default"]);
            para_.lidar_setting_.cut_corner = pt.get<int>("Setting.cut_corner", numb_map["cut_corner"]["default"]);
            para_.lidar_setting_.maxfps = pt.get<int>("Setting.maxfps", numb_map["maxfps"]["default"]);

            para_.lidar_setting_.start_stream = pt.get<bool>("Setting.start_stream", bool_map["start_stream"]["default"]);
            para_.lidar_setting_.connect_address = pt.get<std::string>("Setting.connect_address", general_map["connect_address"]);

            para_.lidar_setting_.hmirror = pt.get<bool>("Setting.hmirror", false);
            para_.lidar_setting_.vmirror = pt.get<bool>("Setting.vmirror", false);
            para_.lidar_setting_.usb_com = pt.get<bool>("Setting.usb_com", bool_map["usb_com"]["default"]);
            para_.lidar_setting_.gray_on = pt.get<bool>("Setting.gray_on", bool_map["gray_on"]["default"]);
            para_.lidar_setting_.is_use_devconfig = pt.get<bool>("Setting.is_use_devconfig", bool_map["is_use_devconfig"]["default"]);
            para_.lidar_setting_.usb_com_name = pt.get<std::string>("Setting.vmirror", general_map["usb_com_name"]);

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

            para_.lidar_ros_.frame_id = pt.get<std::string>("Setting.frame_id", general_map["frame_id"]);
            para_.lidar_ros_.topic_name = pt.get<std::string>("Setting.topic_name", general_map["topic_name"]);
        }
        else
        {
            para_.lidar_setting_.freq = enum_map["freq"]["default"];
            para_.lidar_setting_.freq1 = enum_map["freq1"]["default"];
            para_.lidar_setting_.freq2 = enum_map["freq2"]["default"];
            para_.lidar_setting_.freq3 = enum_map["freq3"]["default"];
            para_.lidar_setting_.freq4 = enum_map["freq4"]["default"];
            para_.lidar_setting_.HDR = enum_map["HDR"]["default"];
            para_.lidar_setting_.imgType = enum_map["imgType"]["default"];
            para_.lidar_setting_.cloud_coord = enum_map["cloud_coord"]["default"];

            para_.lidar_setting_.int1 = numb_map["int1"]["default"];
            para_.lidar_setting_.int2 = numb_map["int2"]["default"];
            para_.lidar_setting_.int3 = numb_map["int3"]["default"];
            para_.lidar_setting_.int4 = numb_map["int4"]["default"];
            para_.lidar_setting_.intgs = numb_map["intgs"]["default"];
            para_.lidar_setting_.minLSB = numb_map["minLSB"]["default"];
            para_.lidar_setting_.cut_corner = numb_map["cut_corner"]["default"];
            para_.lidar_setting_.maxfps = numb_map["maxfps"]["default"];

            para_.lidar_setting_.start_stream = bool_map["start_stream"]["default"];
            para_.lidar_setting_.connect_address = general_map["connect_address"];

            para_.lidar_setting_.usb_com = bool_map["usb_com"]["default"];
            para_.lidar_setting_.gray_on = bool_map["gray_on"]["default"];
            para_.lidar_setting_.is_use_devconfig = bool_map["is_use_devconfig"]["default"];
            para_.lidar_setting_.usb_com_name = general_map["usb_com_name"];

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
            para_.lidar_filter_.dustThreshold = 2000;
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

            para_.lidar_ros_.frame_id = general_map["frame_id"];
            para_.lidar_ros_.topic_name = general_map["topic_name"];
        }
    }
    void list_and_print_parameters(const std::string &prefix)
    {
        // List parameters with the given prefix
        auto result = this->list_parameters({prefix}, 10); // Depth of 10 for example

        // Print parameter names and values
        for (const auto &param_name : result.names)
        {
            rclcpp::Parameter parameter;
            if (this->get_parameter(param_name, parameter))
            {
                RCLCPP_INFO(this->get_logger(), "Parameter name: %s, value: %s",
                            param_name.c_str(), parameter.value_to_string().c_str());
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Parameter '%s' not found", param_name.c_str());
            }
        }

        // Recursively list and print parameters in sub-namespaces
        for (const auto &namespace_name : result.prefixes)
        {
            list_and_print_parameters(namespace_name);
        }
    }

    rcl_interfaces::msg::SetParametersResult SetParametersCallback(const std::vector<rclcpp::Parameter> &parameters, para_ros *para_)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        for (auto &param : parameters)
        {
            RCLCPP_INFO(this->get_logger(), "param %s update", param.get_name().c_str());
            // if (is_connected == 0)
            // {
            //     std::cout << "Waiting for connection" << std::endl;
            //     if (param.get_name() != "usb_com")
            //     {
            //         return result;
            //     }
            // }

            if (param.get_name() == "usb_com_name")
            {
                para_->lidar_setting_.usb_com_name = param.as_string();
                if (para_->lidar_setting_.usb_com)
                {
                    if (para_->lidar_setting_.usb_com_name != "")
                    {
                        xtsdk->setConnectSerialportName(para_->lidar_setting_.usb_com_name);
                    }
                    else
                    {
                        RCLCPP_ERROR(this->get_logger(), "Invalid usb_com_name");
                    }
                }

                // std::cout << "SET usb_com TO: " << pdevparam->usb_com << std::endl;
            }
            else if (param.get_name() == "usb_com")
            {
                para_->lidar_setting_.usb_com = param.as_bool();
                if (para_->lidar_setting_.usb_com)
                {
                    if (para_->lidar_setting_.usb_com_name != "")
                    {
                        xtsdk->stop();
                        xtsdk->setConnectSerialportName(para_->lidar_setting_.usb_com_name);
                        xtsdk->start((ImageType)para_->lidar_setting_.imgType);
                    }
                    else
                    {
                        RCLCPP_ERROR(this->get_logger(), "Invalid usb_com_name");
                    }
                }
                else
                {
                    xtsdk->stop();
                    xtsdk->setConnectIpaddress(para_->lidar_setting_.connect_address);
                    xtsdk->start((ImageType)para_->lidar_setting_.imgType);
                }

                // std::cout << "SET usb_com TO: " << pdevparam->usb_com << std::endl;
            }
            else if (param.get_name() == "connect_address")
            {
                para_->lidar_setting_.connect_address = param.as_string();
                std::cout << para_->lidar_setting_.connect_address << std::endl;
                if (!para_->lidar_setting_.usb_com)
                {
                    xtsdk->setConnectIpaddress(para_->lidar_setting_.connect_address);
                    // xtsdk->start((ImageType)para_->lidar_setting_.imgType);
                }
                else
                {
                    RCLCPP_WARN(this->get_logger(), "USB MODE SET");
                }

                // std::cout << "SET connect_address TO: " << connect_address << std::endl;
            }
            else if (param.get_name() == "imgType")
            {

                para_->lidar_setting_.imgType = param.as_int();
                if (is_connected == 0)
                {
                    std::cout << "maxfps NOT SET TO SDK " << std::endl;
                    continue;
                }
                if (para_->lidar_setting_.start_stream)
                {

                    xtsdk->start((ImageType)para_->lidar_setting_.imgType);
                }
                else
                {
                    xtsdk->stop();
                }
                // std::cout << "SET image_type TO: " << (ImageType)pdevparam->image_type << std::endl;
            }
            else if (param.get_name() == "HDR")
            {

                para_->lidar_setting_.HDR = param.as_int();
                if (is_connected == 0)
                {
                    std::cout << "HDR NOT SET TO SDK " << std::endl;
                    continue;
                }
                xtsdk->setHdrMode((HDRMode)(para_->lidar_setting_.HDR));
            }
            else if (param.get_name() == "cloud_coord")
            {

                para_->lidar_setting_.cloud_coord = param.as_int();
                xtsdk->setSdkCloudCoordType((ClOUDCOORD_TYPE)(para_->lidar_setting_.cloud_coord));
            }
            else if (param.get_name() == "int1")
            {
                para_->lidar_setting_.int1 = param.as_int();
                if (is_connected == 0)
                {
                    std::cout << "int1 NOT SET TO SDK " << std::endl;
                    continue;
                }
                xtsdk->setIntTimesus(para_->lidar_setting_.intgs,
                                     para_->lidar_setting_.int1,
                                     para_->lidar_setting_.int2,
                                     para_->lidar_setting_.int3,
                                     para_->lidar_setting_.int4);
                // std::cout << "SET integration_time_tof_1 TO: " << pdevparam->int1 << std::endl;
            }
            else if (param.get_name() == "int2")
            {

                para_->lidar_setting_.int2 = param.as_int();
                if (is_connected == 0)
                {
                    std::cout << "int2 NOT SET TO SDK " << std::endl;
                    continue;
                }
                xtsdk->setIntTimesus(para_->lidar_setting_.intgs,
                                     para_->lidar_setting_.int1,
                                     para_->lidar_setting_.int2,
                                     para_->lidar_setting_.int3,
                                     para_->lidar_setting_.int4);
                // std::cout << "SET integration_time_tof_2 TO: " << pdevparam->int2 << std::endl;
            }
            else if (param.get_name() == "int3")
            {

                para_->lidar_setting_.int3 = param.as_int();
                if (is_connected == 0)
                {
                    std::cout << "int3 NOT SET TO SDK " << std::endl;
                    continue;
                }
                xtsdk->setIntTimesus(para_->lidar_setting_.intgs,
                                     para_->lidar_setting_.int1,
                                     para_->lidar_setting_.int2,
                                     para_->lidar_setting_.int3,
                                     para_->lidar_setting_.int4);
                // std::cout << "SET integration_time_tof_3 TO: " << pdevparam->int3 << std::endl;
            }
            else if (param.get_name() == "int4")
            {

                para_->lidar_setting_.int4 = param.as_int();
                if (is_connected == 0)
                {
                    std::cout << "int4 NOT SET TO SDK " << std::endl;
                    continue;
                }
                xtsdk->setIntTimesus(para_->lidar_setting_.intgs,
                                     para_->lidar_setting_.int1,
                                     para_->lidar_setting_.int2,
                                     para_->lidar_setting_.int3,
                                     para_->lidar_setting_.int4);
                // std::cout << "SET integration_time_tof_3 TO: " << pdevparam->int3 << std::endl;
            }
            else if (param.get_name() == "intgs")
            {

                para_->lidar_setting_.intgs = param.as_int();
                if (is_connected == 0)
                {
                    std::cout << "intgs NOT SET TO SDK " << std::endl;
                    continue;
                }
                xtsdk->setIntTimesus(para_->lidar_setting_.intgs,
                                     para_->lidar_setting_.int1,
                                     para_->lidar_setting_.int2,
                                     para_->lidar_setting_.int3,
                                     para_->lidar_setting_.int4);
                // std::cout << "SET integration_time_gray TO: " << pdevparam->intgs << std::endl;
            }
            else if (param.get_name() == "minLSB")
            {

                para_->lidar_setting_.minLSB = param.as_int();
                if (is_connected == 0)
                {
                    std::cout << "minLSB NOT SET TO SDK " << std::endl;
                    continue;
                }
                xtsdk->setMinAmplitude(para_->lidar_setting_.minLSB);
                // std::cout << "SET min_amplitude TO: " << pdevparam->minamp << std::endl;
            }
            else if (param.get_name() == "cut_corner")
            {
                para_->lidar_setting_.cut_corner = param.as_int();
                xtsdk->setCutCorner(para_->lidar_setting_.cut_corner);
                // std::cout << "SET cut_corner TO: " << pdevparam->cut_corner << std::endl;
            }
            else if (param.get_name() == "freq")
            {

                para_->lidar_setting_.freq = param.as_int();
                if (is_connected == 0)
                {
                    std::cout << "freq NOT SET TO SDK " << std::endl;
                    continue;
                }
                if (fwversionf >= 2.0)
                {
                    std::cout << "High fw and freq NOT SET TO SDK " << std::endl;
                    continue;
                }
                xtsdk->setModFreq((ModulationFreq)para_->lidar_setting_.freq);
                // std::cout << "SET freq TO: " << pdevparam->freqtype << std::endl;
            }

            else if (param.get_name() == "freq1")
            {

                para_->lidar_setting_.freq1 = param.as_int();
                if (is_connected == 0)
                {
                    std::cout << "freq1 NOT SET TO SDK " << std::endl;
                    continue;
                }
                xtsdk->setMultiModFreq((ModulationFreq)para_->lidar_setting_.freq1,
                                       (ModulationFreq)para_->lidar_setting_.freq2,
                                       (ModulationFreq)para_->lidar_setting_.freq3,
                                       (ModulationFreq)para_->lidar_setting_.freq4);
                // std::cout << "SET freq TO: " << pdevparam->freqtype << std::endl;
            }
            else if (param.get_name() == "freq2")
            {

                para_->lidar_setting_.freq2 = param.as_int();
                if (is_connected == 0)
                {
                    std::cout << "freq NOT SET TO SDK " << std::endl;
                    continue;
                }
                xtsdk->setMultiModFreq((ModulationFreq)para_->lidar_setting_.freq1,
                                       (ModulationFreq)para_->lidar_setting_.freq2,
                                       (ModulationFreq)para_->lidar_setting_.freq3,
                                       (ModulationFreq)para_->lidar_setting_.freq4);
                // std::cout << "SET freq TO: " << pdevparam->freqtype << std::endl;
            }
            else if (param.get_name() == "freq3")
            {

                para_->lidar_setting_.freq3 = param.as_int();
                if (is_connected == 0)
                {
                    std::cout << "freq NOT SET TO SDK " << std::endl;
                    continue;
                }
                xtsdk->setMultiModFreq((ModulationFreq)para_->lidar_setting_.freq1,
                                       (ModulationFreq)para_->lidar_setting_.freq2,
                                       (ModulationFreq)para_->lidar_setting_.freq3,
                                       (ModulationFreq)para_->lidar_setting_.freq4);
                // std::cout << "SET freq TO: " << pdevparam->freqtype << std::endl;
            }
            else if (param.get_name() == "freq4")
            {

                para_->lidar_setting_.freq4 = param.as_int();
                if (is_connected == 0)
                {
                    std::cout << "freq NOT SET TO SDK " << std::endl;
                    continue;
                }
                xtsdk->setMultiModFreq((ModulationFreq)para_->lidar_setting_.freq1,
                                       (ModulationFreq)para_->lidar_setting_.freq2,
                                       (ModulationFreq)para_->lidar_setting_.freq3,
                                       (ModulationFreq)para_->lidar_setting_.freq4);
                // std::cout << "SET freq TO: " << pdevparam->freqtype << std::endl;
            }
            else if (param.get_name() == "start_stream")
            {
                para_->lidar_setting_.start_stream = param.as_bool();
                if (is_connected == 0)
                {
                    std::cout << "start_stream NOT SET TO SDK " << std::endl;
                    continue;
                }
                if (para_->lidar_setting_.start_stream)
                {

                    xtsdk->start((ImageType)para_->lidar_setting_.imgType);
                }
                else
                {
                    xtsdk->stop();
                }
                // std::cout << "SET start_stream TO: " << pdevparam->start_stream << std::endl;
            }
            else if (param.get_name() == "gray_on")
            {
                para_->lidar_setting_.gray_on = param.as_bool();
                if (is_connected == 0)
                {
                    std::cout << "gray_on NOT SET TO SDK " << std::endl;
                    continue;
                }
                if (para_->lidar_setting_.gray_on)
                {

                    xtsdk->setAdditionalGray(1);
                }
                else
                {
                    xtsdk->setAdditionalGray(0);
                }
            }
            else if (param.get_name() == "is_use_devconfig")
            {
                para_->lidar_setting_.is_use_devconfig = param.as_bool();
            }
            else if (param.get_name() == "maxfps")
            {
                para_->lidar_setting_.maxfps = param.as_int();
                if (is_connected == 0)
                {
                    std::cout << "maxfps NOT SET TO SDK " << std::endl;
                    continue;
                }

                xtsdk->setMaxFps((ModulationFreq)para_->lidar_setting_.maxfps);
                // std::cout << "SET connect_address TO: " << connect_address << std::endl;
            }
        }

        return result;
    }

    // 动态参数回到句柄
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
    // 参数描述
    rcl_interfaces::msg::ParameterDescriptor descriptor;

    std::map<std::string, std::map<std::string, int>> enum_map;
    std::map<std::string, std::map<std::string, int>> numb_map;
    std::map<std::string, bool> bool_map;
    std::map<std::string, std::string> general_map;
};

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

// sdk回调处理
void eventCallback(const std::shared_ptr<CBEventData> &event)
{
    auto message = std_msgs::msg::String();
    message.data = "event: " + event->eventstr + " " + std::to_string(event->cmdid);
    eventPublisher->publish(message);

    if (event->eventstr == "sdkState")
    {
        std::cout << "devstate= " + xtsdk->getStateStr() << std::endl;
        XTAPPLOG("devstate= " + xtsdk->getStateStr());
        if (xtsdk->isconnect() && (event->cmdid == 0xfe)) // 端口打开后第一次连接上设备
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
                    std::cout << "******************* GET CONFIG SUCCESS *******************" << std::endl;
                    config_device.lidar_setting_.freq1 = getMultiFreq(static_cast<int>(devconfig.freq[0]));
                    config_device.lidar_setting_.freq2 = getMultiFreq(static_cast<int>(devconfig.freq[1]));
                    config_device.lidar_setting_.freq3 = getMultiFreq(static_cast<int>(devconfig.freq[2]));
                    config_device.lidar_setting_.freq4 = getMultiFreq(static_cast<int>(devconfig.freq[3]));
                    config_device.lidar_setting_.freq = static_cast<int>(devconfig.modFreq);

                    config_device.lidar_setting_.int1 = static_cast<int>(devconfig.integrationTimes[0]);
                    config_device.lidar_setting_.int2 = static_cast<int>(devconfig.integrationTimes[1]);
                    config_device.lidar_setting_.int3 = static_cast<int>(devconfig.integrationTimes[2]);
                    config_device.lidar_setting_.int4 = static_cast<int>(devconfig.integrationTimes[3]);
                    config_device.lidar_setting_.intgs = static_cast<int>(devconfig.integrationTimeGs);

                    config_device.lidar_setting_.imgType = static_cast<int>(4);
                    config_device.lidar_setting_.HDR = static_cast<int>(devconfig.hdrMode);
                    config_device.lidar_setting_.maxfps = static_cast<int>(devconfig.maxfps);
                    config_device.lidar_setting_.minLSB = static_cast<int>(devconfig.miniAmp);

                    std::cout << "version: " << std::to_string(devconfig.version) << std::endl;
                    std::cout << "freq: " << ModulationFreqStr[config_device.lidar_setting_.freq1] << " "
                              << ModulationFreqStr[config_device.lidar_setting_.freq2] << " "
                              << ModulationFreqStr[config_device.lidar_setting_.freq3] << " "
                              << ModulationFreqStr[config_device.lidar_setting_.freq4] << std::endl;
                    std::cout << "ImageType: " << config_device.lidar_setting_.imgType << std::endl;
                    std::cout << "ModulationFreq: " << config_device.lidar_setting_.freq << std::endl;
                    std::cout << "HDRMode: " << config_device.lidar_setting_.HDR << std::endl;
                    std::cout << "integrationTimes: " << config_device.lidar_setting_.int1 << " "
                              << config_device.lidar_setting_.int2 << " "
                              << config_device.lidar_setting_.int3 << " "
                              << config_device.lidar_setting_.int4 << std::endl;
                    std::cout << "integrationTimeGs : " << config_device.lidar_setting_.intgs << std::endl;
                    std::cout << "miniAmp: " << config_device.lidar_setting_.minLSB << std::endl;
                    std::cout << "setmaxfps: " << config_device.lidar_setting_.maxfps << std::endl;

                    std::cout << "isFilterOn: "
                              << std::to_string(devconfig.isFilterOn)
                              << std::endl;
                    std::cout << "roi: " << std::to_string(devconfig.roi[0])
                              << " " << std::to_string(devconfig.roi[1]) << " "
                              << std::to_string(devconfig.roi[2]) << " "
                              << std::to_string(devconfig.roi[3]) << std::endl;
                    std::cout << "maxfps: " << std::to_string(devconfig.maxfps) << std::endl;
                    std::cout << "bCompensateOn: " << std::to_string(devconfig.bCompensateOn) << std::endl;
                    std::cout << "bBinningH: " << std::to_string(devconfig.bBinningH) << std::endl;
                    std::cout << "bBinningV: " << std::to_string(devconfig.bBinningV) << std::endl;
                    std::cout << "freqChannel: " << std::to_string(devconfig.freqChannel) << std::endl;

                    std::cout << "endianType: " << std::to_string(devconfig.endianType) << std::endl;

                    std::cout << "bcut_filteron: " << std::to_string(devconfig.bcut_filteron) << std::endl;
                    std::cout << "cut_intgrttime0: " << std::to_string(devconfig.cut_intgrttime0) << std::endl;
                    std::cout << "cut_distance0: " << std::to_string(devconfig.cut_distance0) << std::endl;
                    std::cout << "cut_intgrttime1: " << std::to_string(devconfig.cut_intgrttime1) << std::endl;
                    std::cout << "cut_distance1: " << std::to_string(devconfig.cut_distance1) << std::endl;
                    std::cout << "********************************************************" << std::endl;
                }
            }
            is_connected += 1;
            is_connected = is_connected > 10 ? 10 : is_connected;
            std::this_thread::sleep_for(std::chrono::seconds(1));

            if (is_connected > 1)
            {
                updateParaFromDev();
            }
        }
    }
}

void imgCallback(const std::shared_ptr<Frame> &frame)
{
    if (is_connected == 0)
    {
        RCLCPP_INFO(g_node->get_logger(), "not connected to lidar, skip image");
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
                RCLCPP_ERROR(g_node->get_logger(), "SYNC LOST");
                check_count = 0;
            }
        }
        else
        {
            if (check_flag)
            {
                RCLCPP_INFO(g_node->get_logger(), "SYNC");
                check_count = 0;
            }
        }
    }
    else
    {
        if (check_flag)
        {
            RCLCPP_WARN(g_node->get_logger(), "NO SYNC");
            check_count = 0;
        }
    }
    rclcpp::Time cloud_time(sec, frame->timeStampNS);
    const size_t nPixel = frame->width * frame->height;
    pcl::PointCloud<pcl::PointXYZI> cloud;
    cloud.header.frame_id = para_set.lidar_ros_.frame_id;
    cloud.header.stamp = pcl_conversions::toPCL(cloud_time);
    cloud.width = static_cast<uint32_t>(frame->width);
    cloud.height = static_cast<uint32_t>(frame->height);
    cloud.is_dense = false;
    cloud.points.resize(nPixel);
    distance_tmp.resize(nPixel);
    amplitude_tmp.resize(nPixel);
    gray_tmp.resize(nPixel);

    for (int i = 0; i < (int)nPixel; i++)
    {
        if (frame->hasPointcloud)
        {
            pcl::PointXYZI &p = cloud.points[i];
            p.x = frame->points[i].x;
            p.y = frame->points[i].y;
            p.z = frame->points[i].z;
            p.intensity = frame->amplData[i] > AMPLITUDE_ABNORMAL ? 0 : frame->amplData[i];
        }

        distance_tmp[i] = frame->distData[i] > DEPTH_ABNORMAL ? 0 : frame->distData[i];
        amplitude_tmp[i] = frame->amplData[i] > AMPLITUDE_ABNORMAL ? 0 : frame->amplData[i];
        if (frame->info.imageflags & IMG_GS16)
        {
            gray_tmp[i] = frame->grayscaledata[i];
        }
    }

    std::cout << "img: " + std::to_string(frame->frame_id) << std::endl;
    sensor_msgs::msg::PointCloud2 cloudout;
    pcl::toROSMsg(cloud, cloudout);
    pointCloud2Publisher->publish(cloudout);

    // }
    if (frame->dataType == Frame::DISTANCE || frame->dataType == Frame::AMPLITUDE)
    {
        sensor_msgs::msg::Image imgDistance;
        imgDistance.header.stamp = cloud_time;
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
        distanceImagePublisher->publish(imgDistance);
    }
    if (frame->dataType == Frame::AMPLITUDE)
    {
        sensor_msgs::msg::Image imgAmpl;
        imgAmpl.header.stamp = cloud_time;
        imgAmpl.header.frame_id = para_set.lidar_ros_.frame_id;
        imgAmpl.height = static_cast<uint32_t>(frame->height);
        imgAmpl.width = static_cast<uint32_t>(frame->width);
        imgAmpl.encoding = sensor_msgs::image_encodings::MONO16;
        imgAmpl.step = imgAmpl.width * frame->px_size;
        imgAmpl.is_bigendian = 1;

        // imgAmpl.data.assign((uint8_t *)(frame->amplData.data()), (uint8_t *)(frame->amplData.data()) + frame->amplData.size() * 2);
        imgAmpl.data.assign((uint8_t *)(amplitude_tmp.data()), (uint8_t *)(amplitude_tmp.data()) + amplitude_tmp.size() * 2);
        amplitudeImagePublisher->publish(imgAmpl);
    }
    if (frame->info.imageflags & IMG_GS16)
    {
        sensor_msgs::msg::Image imgGray;
        imgGray.header.stamp = cloud_time;
        imgGray.header.frame_id = para_set.lidar_ros_.frame_id;
        imgGray.height = static_cast<uint32_t>(frame->height);
        imgGray.width = static_cast<uint32_t>(frame->width);
        imgGray.encoding = sensor_msgs::image_encodings::MONO16;
        imgGray.step = imgGray.width * frame->px_size;
        imgGray.is_bigendian = 1;

        // imgAmpl.data.assign((uint8_t *)(frame->amplData.data()), (uint8_t
        // *)(frame->amplData.data()) + frame->amplData.size() * 2);
        imgGray.data.assign((uint8_t *)(gray_tmp.data()), (uint8_t *)(gray_tmp.data()) + gray_tmp.size() * 2);
        grayImagePublisher->publish(imgGray);
    }
}

void signalHandler(int signum)
{
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Received signal: %d", signum);
    rclcpp::shutdown(); // 触发节点关闭
}

int main(int argc, char **argv)
{
    xtsdk = new XtSdk();
    xtsdk->setCallback(eventCallback, imgCallback);

    rclcpp::init(argc, argv);
    g_node = std::make_shared<XtsdkNode>();

    // xtsdk->setConnectIpaddress(para_set.lidar_setting_.connect_address);
    // xtsdk->setConnectSerialportName("COM3");//ubuntu 下设备名是 "/dev/ttyACM0"
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

    xtsdk->setCutCorner(para_set.lidar_setting_.cut_corner);
    xtsdk->startup();

    std::signal(SIGINT, signalHandler);
    while (rclcpp::ok() && is_connected == 0)
    {
        rclcpp::spin_some(g_node);
    }

    updateParaFromDev();
    std::this_thread::sleep_for(std::chrono::duration<double>(0.5));
    print_curr_para();

    // 开始处理循环
    rclcpp::spin(g_node);

    // 清理资源
    xtsdk->stop();
    xtsdk->setCallback();
    xtsdk->shutdown();

    rclcpp::shutdown();
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "-----------TERMINATED----------");
    return 0;
}
