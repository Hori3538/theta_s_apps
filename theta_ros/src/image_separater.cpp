#include <image_separater/image_separater.h>

namespace theta_ros
{
    ImageSeparater::ImageSeparater(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    {
        pnh.param("theta_topic_name", theta_topic_name_, std::string("/theta/image_raw"));

        image_transport::ImageTransport it(nh);
        image_sub_ = it.subscribe(theta_topic_name_, 1, &ImageSeparater::image_callback, this);
    }

    void ImageSeparater::image_callback(const sensor_msgs::ImageConstPtr &msg)
    {

        cv_bridge::CvImagePtr cv_ptr;
        try{
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            input_image_ = cv_ptr->image;

            cv::Mat splitted_image1, splitted_image2;
            split_image(input_image_, splitted_image1, splitted_image2);

            // cv::imshow("theta_image", input_image_);
            // cv::waitKey();
        }
        catch(cv_bridge::Exception &e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }
    void ImageSeparater::split_image(cv::Mat& origin_image, cv::Mat& splitted_image1,
            cv::Mat& splitted_image2)
    {
        // cv::imshow("origin_image", origin_image);
        // cv::waitKey();
        int height = origin_image.rows;
        int width = origin_image.cols/2;

        // 720*1280*1 -> 640*640*2
        splitted_image1 = origin_image(cv::Rect(0, 0, width, width));
        splitted_image2 = origin_image(cv::Rect(width, 0, width, width));

        // 画像の向きを修正
        cv::rotate(splitted_image1, splitted_image1, cv::ROTATE_90_CLOCKWISE);
        cv::rotate(splitted_image2, splitted_image2, cv::ROTATE_90_COUNTERCLOCKWISE);

        cv::imshow("splitted_image1", splitted_image1);
        cv::waitKey(5);
        // cv::imshow("splitted_image2", splitted_image2);
        // cv::waitKey();
    }

}
