#ifndef BAG_HANDLER_H
#define BAG_HANDLER_H

#include <iostream>
#include <string>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <regex>

#if __cplusplus >= 201703L
#include <filesystem>
namespace fs = std::filesystem;
#else
#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;
#endif

#if __has_include(<ros/ros.h>)
#define ROS1
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <sensor_msgs/PointCloud2.h>
#elif __has_include(<rclcpp/rclcpp.hpp>)
#define ROS2
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_cpp/storage_options.hpp>
#include <rosbag2_cpp/converter_options.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rosbag2_storage_default_plugins/sqlite/sqlite_storage.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rosbag2_storage/topic_metadata.hpp>
#else
#error "Neither ROS 1 nor ROS 2 environment found. Please source the appropriate setup script."
#endif

class BagHandler
{
public:
    BagHandler(const std::string &folder_path, const std::string &bag_file, const std::string &topic_name);
    ~BagHandler() = default;

    void convertPcdToBag();
    std::vector<std::pair<int, std::string>> getFiles(const std::string &folder_path);

private:
    pcl::PointCloud<pcl::PointXYZI> cloud_;
    std::vector<std::pair<int, std::string>> _pcd_files;
    std::string _topic_name;
    std::string _bag_file;
#if defined(ROS2)
    rclcpp::Time getCurrentTime();
#endif
};
BagHandler::BagHandler(const std::string &folder_path, const std::string &bag_file, const std::string &topic_name)
{
    _pcd_files = getFiles(folder_path);
    _topic_name = topic_name;
    _bag_file = bag_file;
}
void BagHandler::convertPcdToBag()
{
#if defined(ROS1)
    rosbag::Bag bag;
    try
    {
        bag.open(_bag_file, rosbag::bagmode::Write);
    }
    catch (rosbag::BagException &e)
    {
        ROS_ERROR("Error opening bag file: %s", e.what());
        return;
    }
#elif defined(ROS2)
    rosbag2_cpp::StorageOptions storage_options;
    storage_options.uri = _bag_file;
    storage_options.storage_id = "sqlite3"; // Use 'sqlite3' or any other storage plugin you are using

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr"; // Default for ROS2
    converter_options.output_serialization_format = "cdr";
    // rosbag2_cpp::Writer writer;

    // Create Writer with shared pointer to interface

    // auto writer = std::make_shared<rosbag2_cpp::Writer>(std::make_shared<rosbag2_cpp::Writer>());
    auto writer = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();
    try
    {
        writer->open(storage_options, converter_options);
    }
    catch (const std::runtime_error &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Error opening bag file: %s", e.what());
        return;
    }
    writer->create_topic({_topic_name, "sensor_msgs/msg/PointCloud2", "cdr"});
#endif

    for (const auto &file : _pcd_files)
    {
        std::cout << "file number: " << file.first << std::endl;
        if (pcl::io::loadPCDFile(file.second, cloud_) == -1)
        {
            std::cerr << "Couldn't read PCD file: " << file.second << std::endl;
            continue;
        }

#if defined(ROS1)
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud_, cloud_msg);
        cloud_msg.header.frame_id = "map";
        bag.write(_topic_name, ros::Time::now(), cloud_msg);
#elif defined(ROS2)
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(cloud_, output);
        output.header.frame_id = "map";
        output.header.stamp = getCurrentTime();
        rosbag2_storage::TopicMetadata topic_metadata;
        topic_metadata.name = _topic_name;
        topic_metadata.type = "sensor_msgs/msg/PointCloud2";
        topic_metadata.serialization_format = "cdr";

        writer->create_topic(topic_metadata);

        // 创建一个 PointCloud2 消息作为示例
        auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        cloud_msg->header.stamp = rclcpp::Clock().now(); // 设置时间戳
        cloud_msg->header.frame_id = "map";              // 设置帧 ID

        // 序列化消息
        rclcpp::Serialization<sensor_msgs::msg::PointCloud2> serializer;
        rclcpp::SerializedMessage serialized_msg;
        serializer.serialize_message(&output, &serialized_msg);

        // 创建并填充 SerializedBagMessage
        auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();
        bag_message->topic_name = topic_metadata.name;
        bag_message->serialized_data = std::make_shared<rcutils_uint8_array_t>(serialized_msg.release_rcl_serialized_message());
        bag_message->time_stamp = output.header.stamp.nanosec;

        // 写入消息
        writer->write(bag_message);

#endif
        std::cout << "Converted PCD file to ROS bag file: " << file.second << std::endl;
    }
#if defined(ROS1)
    bag.close();
#endif
}

std::vector<std::pair<int, std::string>> BagHandler::getFiles(const std::string &folder_path)
{
    fs::path path(folder_path);
    fs::directory_iterator end_iter;
    std::vector<std::pair<int, std::string>> filesWithNumbers;

    // 正则表达式，用于从文件名中提取数字
    std::regex re("(\\d+)");
    for (const auto &entry : fs::directory_iterator(folder_path))
    {

        std::string path = entry.path().string();

        if (entry.path().extension() == ".pcd")
        {
            std::smatch match;
            std::string filename = entry.path().stem().string();
            if (std::regex_search(filename, match, re) && match.size() > 1)
            {
                for (size_t i = 0; i < match.size(); ++i)
                {
                    std::cout << "Match[" << i << "]: " << match.str(i) << std::endl;
                }
                // 将数字和文件路径作为一对保存
                int number = std::stoi(match.str(1));
                // std::cout << number << std::endl;
                filesWithNumbers.emplace_back(number, path);
                std::cout << "Matched number: " << match[1].str() << " in filename: " << path << std::endl;
            }
        }
    }
    std::sort(filesWithNumbers.begin(), filesWithNumbers.end(),
              [](const std::pair<int, std::string> &a, const std::pair<int, std::string> &b)
              { return a.first < b.first; });
    return filesWithNumbers;
}

#if defined(ROS2)
rclcpp::Time BagHandler::getCurrentTime()
{
    auto now = std::chrono::system_clock::now();
    auto now_ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);
    auto epoch = now_ns.time_since_epoch();
    auto value = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch);
    return rclcpp::Time(value.count());
}
#endif
#endif // BAG_HANDLER_H
