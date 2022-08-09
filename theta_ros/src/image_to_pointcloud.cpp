#include <image_to_pointcloud/image_to_pointcloud.h>

namespace theta_ros
{
    ImageToPointcloud::ImageToPointcloud(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    {
        pnh.param("equirectangular_topic_name", equirectangular_topic_name_, std::string("/equirectangular/image_raw"));
        pnh.param("depth_topic_name", depth_topic_name_, std::string("/depth_image"));
        pnh.param("pc_frame_name", pc_frame_name_, std::string("camera_fixed_frame"));

        equirectangular_sub_  = new message_filters::Subscriber<sensor_msgs::Image> (nh, equirectangular_topic_name_, 5);
        depth_sub_ = new message_filters::Subscriber<sensor_msgs::Image>(nh, depth_topic_name_, 5);
        sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), *equirectangular_sub_, *depth_sub_);
        sync_->registerCallback(&ImageToPointcloud::sync_callback, this);

        pc_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/theta_pointcloud", 1);
    }

    void ImageToPointcloud::sync_callback(const sensor_msgs::ImageConstPtr &equirectangular_msg,
            const sensor_msgs::ImageConstPtr &depth_msg)
    {
        cv_bridge::CvImagePtr equirectangular_ptr;
        cv_bridge::CvImagePtr depth_ptr;
        try{
            equirectangular_ptr = cv_bridge::toCvCopy(equirectangular_msg, sensor_msgs::image_encodings::BGR8);
            depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::MONO8);
            equirectangular_image_ = equirectangular_ptr->image;
            // cv::imshow("input image", equirectangular_image_);
            // int key = cv::waitKey(5);
            depth_image_ = depth_ptr->image; 
            // cv::imshow("input image", depth_image_);
            // int key = cv::waitKey(5);

            cv::Mat test_img = cv::Mat::ones(equirectangular_image_.rows, equirectangular_image_.cols, CV_32F);
            // pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc = image_to_pointcloud(equirectangular_image_, test_img);
            depth_image_ = cv::Scalar(255) - depth_image_;

            double scale = 0.02;
            double shift = 0.1;
            cv::Mat normalized_depth_image(depth_image_.rows, depth_image_.cols, CV_32F);
            normalize_scale_shift(depth_image_, normalized_depth_image, scale, shift);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc = image_to_pointcloud(equirectangular_image_, normalized_depth_image);


            sensor_msgs::PointCloud2 pc_ros;
            pcl::toROSMsg(*pc, pc_ros);
            pc_ros.header.stamp = equirectangular_msg->header.stamp;
            // pc_ros.header.stamp = ros::Time::now();
            pc_ros.header.frame_id = pc_frame_name_;

            pc_pub_.publish(pc_ros);
        }
        catch(cv_bridge::Exception &e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

    }
    std::pair<double, double> ImageToPointcloud::pixel_to_angular(int px, int py, int width, int height)
    {
        double latitude = 2 * M_PI * px/width;
        double longitude = M_PI * py/height;

        return {latitude, longitude};
    }
    std::tuple<double, double, double> ImageToPointcloud::polar_to_cartesian(double latitude, double longitude, double r)
    {
        double x = -r * std::sin(longitude) * std::cos(latitude);
        double y = r * std::sin(longitude) * std::sin(latitude);
        double z = r * std::cos(longitude);

        return {x, y, z};
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ImageToPointcloud::image_to_pointcloud(cv::Mat equirectangular_image, cv::Mat depth_image)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pc {new pcl::PointCloud<pcl::PointXYZRGB>};
        pc->width = depth_image.cols;
        pc->height = depth_image.rows;

        int width = equirectangular_image.cols;
        // int height = equirectangular_image.rows;
        int height = width / 2;
        for(int py=0; py<height; py++){
            for(int px=0; px<width; px++){
                cv::Vec3b bgr = equirectangular_image.at<cv::Vec3b>(py, px);
                float depth = depth_image.at<float>(py, px);
                auto [latitude, longitude] = pixel_to_angular(px, py, width, height);
                auto [x, y, z] = polar_to_cartesian(latitude, longitude, depth);

                pcl::PointXYZRGB point(bgr[2], bgr[1], bgr[0]);
                point.x = x;
                point.y = y;
                point.z = z;
                pc->points.push_back(point);
            }
        }
        return pc;
    }
    void ImageToPointcloud::normalize_scale_shift(cv::Mat& depth_image, cv::Mat& normalized_depth_image, double scale, double shift)
    {

        //引用 https://github.com/iwatake2222/opencv_sample/blob/master/reconstruction_depth_to_3d/depth_engine.cpp

        /***
        * Normalize to float (Far = huge value, Near = small value)
        * 1 / Normalized Value = Estimated Depth(inverse relative depth) * scale + shift
        ***/
        normalized_depth_image = cv::Mat(depth_image.rows, depth_image.cols, CV_32F);
        depth_image.convertTo(normalized_depth_image, CV_32F, scale, shift);
    }
}
