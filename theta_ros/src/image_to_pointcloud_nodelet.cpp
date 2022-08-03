#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <image_to_pointcloud/image_to_pointcloud.h>


namespace theta_ros{
    class ImageToPointcloudNodelet : public nodelet::Nodelet
    {
        public:
            ImageToPointcloudNodelet() = default;
            ~ImageToPointcloudNodelet() {
        if (image_to_pointcloud_) delete image_to_pointcloud_;
            }
        private:
            virtual void onInit() {
                ros::NodeHandle nh;
                ros::NodeHandle pnh("~");
                pnh = getPrivateNodeHandle();
                image_to_pointcloud_ = new theta_ros::ImageToPointcloud(nh, pnh);
            }
            theta_ros::ImageToPointcloud *image_to_pointcloud_;
    };
}
// Declare as a Plug-in
PLUGINLIB_EXPORT_CLASS(theta_ros::ImageToPointcloudNodelet, nodelet::Nodelet);
