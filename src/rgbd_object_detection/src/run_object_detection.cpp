/// \file
/// \brief Node for object detection and map building
///

#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <rgbd_object_detection/object_detector_v2.hpp>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "run_object_detection");
    ros::NodeHandle nh;
    std::cout << "Outside callback~~" << std::endl;
    disinfection_robot::ObjectDetectorV2 object_detector_v2(nh);
    std::cout << "Outside callback~~ end " << std::endl;
    ros::spin();

    return 0;
}
