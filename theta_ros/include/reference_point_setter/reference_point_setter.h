#ifndef REFERENCE_POINT_SETTER
#define REFERENCE_POINT_SETTER

#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sstream>
#include <fstream>
#include <filesystem>

namespace theta_ros
{
    class ReferencePointSetter
    {
        public:
            ReferencePointSetter(ros::NodeHandle &nh, ros::NodeHandle &pnh);
        private:
            void image_callback(const sensor_msgs::ImageConstPtr &msg);
            static void mouse_callback_static(int event, int x, int y, int flag, void *userdata);
            void mouse_callback(int event, int x, int y, int flag);
            void save_reference_area(cv::Rect reference_rect);

            // parameter
            std::string equirectangular_topic_name_;
            std::string reference_save_dir_;

            cv::Mat input_image_;
            cv::Rect rectangle_value_;
            bool is_click_;
            int reference_point_count_ = 0;

            image_transport::Subscriber image_sub_;

    };
}


#endif
