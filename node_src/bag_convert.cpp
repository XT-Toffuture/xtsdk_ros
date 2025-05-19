#include <iostream>
#include "utility/bag_handler.h"

int main(int argc, char **argv)
{
    if (argc != 4)
    {
        std::cerr << "Usage: " << argv[0] << " <PCD folder path> <output bag file> <topic name>" << std::endl;
        return 1;
    }

    std::string folder_path = argv[1];
    std::string bag_file = argv[2];
    std::string topic_name = argv[3];

#if defined(ROS1)
    ros::init(argc, argv, "pcd_to_bag_converter");
    ros::NodeHandle nh;
#elif defined(ROS2)
    rclcpp::init(argc, argv);
#endif

    BagHandler bag_handler(folder_path, bag_file, topic_name);
    bag_handler.convertPcdToBag();

#if defined(ROS1)
    ros::shutdown();
#elif defined(ROS2)
    rclcpp::shutdown();
#endif

    return 0;
}