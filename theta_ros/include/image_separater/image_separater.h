#ifndef IMAGE_SEPARATER
#define IMAGE_SEPARATER

#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

namespace theta_ros
{
    class ImageSeparater
    {
        public:
            ImageSeparater(ros::NodeHandle &nh, ros::NodeHandle &pnh);
        private:
            void image_callback(const sensor_msgs::ImageConstPtr &msg);
            void split_image(cv::Mat& origin_image, cv::Mat& splitted_image1,
                    cv::Mat& splitted_image2);

            // parameter
            std::string theta_topic_name_;

            cv::Mat input_image_;

            image_transport::Subscriber image_sub_;

    };
}


#endif
