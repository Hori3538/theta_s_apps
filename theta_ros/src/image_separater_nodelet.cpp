#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <image_separater/image_separater.h>


namespace theta_ros{
    class ImageSeparaterNodelet : public nodelet::Nodelet
    {
        public:
            ImageSeparaterNodelet() = default;
            ~ImageSeparaterNodelet() {
        if (image_separater_) delete image_separater_;
            }
        private:
            virtual void onInit() {
                ros::NodeHandle nh;
                ros::NodeHandle pnh("~");
                pnh = getPrivateNodeHandle();
                image_separater_ = new theta_ros::ImageSeparater(nh, pnh);
            }
            theta_ros::ImageSeparater *image_separater_;
    };
}
// Declare as a Plug-in
PLUGINLIB_EXPORT_CLASS(theta_ros::ImageSeparaterNodelet, nodelet::Nodelet);
