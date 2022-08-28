#include <ros/ros.h>
#include <reference_point_setter/reference_point_setter.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "reference_point_setter_node");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    theta_ros::ReferencePointSetter reference_point_setter(nh, pnh);

    ros::spin();
    return 0;
}
