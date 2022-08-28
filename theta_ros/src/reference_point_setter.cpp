#include <reference_point_setter/reference_point_setter.h>

namespace theta_ros
{
    ReferencePointSetter::ReferencePointSetter(ros::NodeHandle &nh, ros::NodeHandle &pnh)
    {
        pnh.param("equirectangular_topic_name", equirectangular_topic_name_, std::string("/theta/image_raw"));
        pnh.param("reference_save_dir", reference_save_dir_, std::string("theta_s_references"));

        image_transport::ImageTransport it(nh);
        image_sub_ = it.subscribe(equirectangular_topic_name_, 1, &ReferencePointSetter::image_callback, this);
    }

    void ReferencePointSetter::image_callback(const sensor_msgs::ImageConstPtr &msg)
    {

        cv_bridge::CvImagePtr cv_ptr;
        try{
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            input_image_ = cv_ptr->image;

            cv::Mat img = input_image_;
            cv::Mat draw_img = img.clone();
            std::string window_name = "example";

            cv::imshow(window_name, img);
            cv::setMouseCallback(window_name, ReferencePointSetter::mouse_callback_static, this);
            while(1){
                // 左ボタンが押されたら描画開始
                if (is_click_ == true) {
                    cv::rectangle(draw_img, rectangle_value_, cv::Scalar(255, 0, 0), 3);
                }

                cv::imshow(window_name, draw_img);
                draw_img = img.clone();

                // qキーが押されたら終了
                int key = cv::waitKey(1);
                if(key == 'q') break;
            }
        }
        catch(cv_bridge::Exception &e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }


    void ReferencePointSetter::mouse_callback_static(int event, int x, int y, int flag, void *userdata)
    {

        ReferencePointSetter* rp_setter = static_cast<ReferencePointSetter*>(userdata);
        rp_setter->mouse_callback(event, x, y, flag);
    }

    void ReferencePointSetter::mouse_callback(int event, int x, int y, int flag)
    {
        is_click_ = flag;
        if (event == cv::EVENT_LBUTTONDOWN) {
            std::cout << "Draw rectangle\n"
                << " start position (x, y) : " << x << ", " << y << std::endl;
            rectangle_value_ = cv::Rect(x, y, 0, 0);
        }
        if (event == cv::EVENT_LBUTTONUP) {
            std::cout << " end   position (x, y) : " << x << ", " << y << std::endl;
            rectangle_value_.width = x - rectangle_value_.x;
            rectangle_value_.height = y - rectangle_value_.y;
            save_reference_area(rectangle_value_);
        }
        if (event == cv::EVENT_MOUSEMOVE) {
            if (flag) {
                rectangle_value_.width = x - rectangle_value_.x;
                rectangle_value_.height = y - rectangle_value_.y;
            }
        }
    }
    void ReferencePointSetter::save_reference_area(cv::Rect reference_rect)
    {
        const char *tmp = getenv("HOME");
        std::string home_dir(tmp ? tmp : "");
        if (home_dir.empty()) {
            std::cerr << "[ERROR] No such variable found!" << std::endl;
            exit(EXIT_FAILURE);
        }

        std::string output_dir = home_dir + "/" + reference_save_dir_;
        // bool ret = fs::create_directories(output_dir);
        // bool ret = std::filesystem::create_directories(output_dir);
        // if(ret) std::cout << "create dir" << std::endl;
        std::ostringstream oss;
        oss << reference_point_count_;
        std::string file_name = "reference_area" + oss.str() + ".txt";

        std::cout << "save dir: " << output_dir + file_name << std::endl;
        std::ofstream fout(output_dir + "/" + file_name);
        if(!fout){
            std::cout << "cannot open the file" << std::endl;
            return;
        }

        // std::cout << "current path: "  << std::filesystem::current_path() << std::endl;

        fout << reference_rect.x << std::endl;
        fout << reference_rect.y << std::endl;
        fout << reference_rect.width << std::endl;
        fout << reference_rect.height << std::endl;

        std::cout << "save " << file_name << std::endl;

        fout.close();
        reference_point_count_++;
    }

}
