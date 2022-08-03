#include <ros/ros.h>
#include <image_to_pointcloud/image_to_pointcloud.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_to_pointcloud_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    theta_ros::ImageToPointcloud image_to_pointcloud(nh, pnh);

    ros::spin();
    return 0;
}
