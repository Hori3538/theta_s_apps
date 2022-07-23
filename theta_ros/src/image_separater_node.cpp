#include <ros/ros.h>
#include <image_separater/image_separater.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_separater_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    theta_ros::ImageSeparater image_separater(nh, pnh);

    ros::spin();
    return 0;
}
