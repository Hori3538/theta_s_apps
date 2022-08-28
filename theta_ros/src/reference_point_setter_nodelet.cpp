#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <reference_point_setter/reference_point_setter.h>


namespace theta_ros{
    class ReferencePointSetterNodelet : public nodelet::Nodelet
    {
        public:
            ReferencePointSetterNodelet() = default;
            ~ReferencePointSetterNodelet() {
        if (reference_point_setter_) delete reference_point_setter_;
            }
        private:
            virtual void onInit() {
                ros::NodeHandle nh;
                ros::NodeHandle pnh("~");
                pnh = getPrivateNodeHandle();
                reference_point_setter_ = new theta_ros::ReferencePointSetter(nh, pnh);
            }
            theta_ros::ReferencePointSetter *reference_point_setter_;
    };
}
// Declare as a Plug-in
PLUGINLIB_EXPORT_CLASS(theta_ros::ReferencePointSetterNodelet, nodelet::Nodelet);
