#ifndef IMAGE_TO_POINTCLOUD
#define IMAGE_TO_POINTCLOUD

#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sstream>

#include <pcl_ros/point_cloud.h>
// #include <pcl/impl/point_types.hpp>
// #include <pcl_ros/transforms.h>
// #include <pcl/common/centroid.h>
// #include <pcl/filters/statistical_outlier_removal.h>
// #include <pcl/filters/voxel_grid.h>
// #include <pcl/segmentation/extract_clusters.h>
// #include <pcl/segmentation/sac_segmentation.h>
// #include <pcl/filters/extract_indices.h>
// #include <pcl/filters/passthrough.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace theta_ros
{
    class ImageToPointcloud
    {
        public:
            ImageToPointcloud(ros::NodeHandle &nh, ros::NodeHandle &pnh);
        private:
            void sync_callback(const sensor_msgs::ImageConstPtr &equirectangular_msg,
                    const sensor_msgs::ImageConstPtr &depth_msg);
            std::pair<double, double> pixel_to_angular(int px, int py, int width, int height);
            std::tuple<double, double, double> polar_to_cartesian(double latitude, double longitude, double r);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr image_to_pointcloud(cv::Mat equirectangular_image, cv::Mat depth_image);
            void normalize_scale_shift(cv::Mat& depth_image, cv::Mat& normalized_depth_image,
                    double scale, double shift);


            // parameter
            std::string equirectangular_topic_name_;
            std::string depth_topic_name_;
            std::string pc_frame_name_;

            cv::Mat equirectangular_image_;
            cv::Mat depth_image_;
            
            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                    sensor_msgs::Image> MySyncPolicy;
            
            message_filters::Subscriber<sensor_msgs::Image> *equirectangular_sub_;
            message_filters::Subscriber<sensor_msgs::Image> *depth_sub_;
            message_filters::Synchronizer<MySyncPolicy> *sync_;

            ros::Publisher pc_pub_;
    };
}


#endif
