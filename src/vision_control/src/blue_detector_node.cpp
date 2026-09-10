#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Point.h>

class BlueDetector {
public:
    BlueDetector() : it_(nh_) {
        // 订阅 usb_cam 原始图像话题
        std::string image_topic;
        nh_.param<std::string>("image_topic", image_topic, "/usb_cam/image_raw");
        image_sub_ = it_.subscribe(image_topic, 1, &BlueDetector::imageCb, this);
        target_pub_ = nh_.advertise<geometry_msgs::Point>("/blue_target", 1);
        debug_pub_ = it_.advertise("/rgb/blue_target_debug_img", 1);

        // ---- 可调 HSV 蓝色范围（现场可用 dynamic_reconfigure 或 rosparam 调整）----
        nh_.param("hsv_low_h", low_h_, 90);
        nh_.param("hsv_high_h", high_h_, 130);
        nh_.param("hsv_low_s", low_s_, 80);
        nh_.param("hsv_high_s", high_s_, 255);
        nh_.param("hsv_low_v", low_v_, 50);
        nh_.param("hsv_high_v", high_v_, 255);
        nh_.param("min_area", min_area_, 800);   // 最小像素面积阈值
        nh_.param("morph_kernel", morph_kernel_, 5); // 开闭运算核大小
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat hsv, mask;
        cv::cvtColor(cv_ptr->image, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(low_h_, low_s_, low_v_), cv::Scalar(high_h_, high_s_, high_v_), mask);

        // 形态学开闭运算降噪
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morph_kernel_, morph_kernel_));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);  // 去除白噪点
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel); // 填补蓝色区域内的黑洞

        // 先找轮廓，取最大轮廓后再算质心，保证红点一定在绿框内
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty()) {
            double max_area = 0;
            int max_idx = 0;
            for (size_t i = 0; i < contours.size(); i++) {
                double area = cv::contourArea(contours[i]);
                if (area > max_area) {
                    max_area = area;
                    max_idx = i;
                }
            }

            // 只对最大轮廓计算质心（用图像矩），避免噪点拉偏质心
            cv::Moments m = cv::moments(contours[max_idx]);
            if (m.m00 > min_area_) {
                double center_x = m.m10 / m.m00;
                double center_y = m.m01 / m.m00;

                geometry_msgs::Point p;
                p.x = center_x / cv_ptr->image.cols; // 归一化 x (0~1)，0.5 代表中心
                p.y = center_y / cv_ptr->image.rows; // 归一化 y (0~1)
                p.z = m.m00 / (cv_ptr->image.cols * cv_ptr->image.rows); // 面积占比给测距
                target_pub_.publish(p);

                // 绿框：最大轮廓的外接矩形
                cv::Rect bounding_rect = cv::boundingRect(contours[max_idx]);
                cv::rectangle(cv_ptr->image, bounding_rect, cv::Scalar(0, 255, 0), 2);
                // 红点：同一轮廓的质心，保证在绿框内
                cv::circle(cv_ptr->image, cv::Point(center_x, center_y), 5, cv::Scalar(0, 0, 255), -1);
            }
        }
        
        // 发布调试图像
        debug_pub_.publish(cv_ptr->toImageMsg());
    }

private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher target_pub_;
    image_transport::Publisher debug_pub_;
    int low_h_, high_h_, low_s_, high_s_, low_v_, high_v_, min_area_, morph_kernel_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "blue_detector_node");
    BlueDetector bd;
    ros::spin();
    return 0;
}
