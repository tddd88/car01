#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <vision_control/parameters.h>
#include <algorithm>

class BlueDetector {
public:
    BlueDetector() : private_nh_("~"), it_(nh_) {
        using vision_control::parameter;
        const std::string topic = parameter(private_nh_, nh_, "image_topic", std::string("/usb_cam/image_raw"));
        image_sub_ = it_.subscribe(topic, 1, &BlueDetector::imageCb, this);
        legacy_pub_ = nh_.advertise<geometry_msgs::Point>("/blue_target", 1);
        target_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>("/vision/target", 1);
        debug_pub_ = it_.advertise("/rgb/blue_target_debug_img", 1);
        low_h_ = parameter(private_nh_, nh_, "hsv_low_h", 90);
        high_h_ = parameter(private_nh_, nh_, "hsv_high_h", 130);
        low_s_ = parameter(private_nh_, nh_, "hsv_low_s", 80);
        high_s_ = parameter(private_nh_, nh_, "hsv_high_s", 255);
        low_v_ = parameter(private_nh_, nh_, "hsv_low_v", 50);
        high_v_ = parameter(private_nh_, nh_, "hsv_high_v", 255);
        min_area_ = parameter(private_nh_, nh_, "min_area", 800);
        morph_kernel_ = parameter(private_nh_, nh_, "morph_kernel", 5);
        if (low_h_ < 0 || high_h_ > 179 || low_h_ > high_h_ ||
            low_s_ < 0 || high_s_ > 255 || low_s_ > high_s_ ||
            low_v_ < 0 || high_v_ > 255 || low_v_ > high_v_ ||
            min_area_ < 0 || morph_kernel_ < 1 || morph_kernel_ > 31) {
            throw std::runtime_error("蓝色 HSV/面积/形态学参数无效，拒绝启动检测");
        }
        kernel_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morph_kernel_, morph_kernel_));
        ROS_INFO("蓝色检测输入 %s，实际 HSV=[%d,%d]/[%d,%d]/[%d,%d] min_area=%d",
                 topic.c_str(), low_h_, high_h_, low_s_, high_s_, low_v_, high_v_, min_area_);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg) {
        cv_bridge::CvImagePtr image;
        try { image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); }
        catch (const cv_bridge::Exception& error) {
            ROS_ERROR_THROTTLE(1.0, "图像转换失败: %s", error.what()); return;
        }
        if (image->image.empty()) return;
        cv::Mat hsv, mask;
        cv::cvtColor(image->image, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(low_h_, low_s_, low_v_), cv::Scalar(high_h_, high_s_, high_v_), mask);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel_);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel_);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        double max_area = 0.0;
        size_t max_index = 0;
        for (size_t i = 0; i < contours.size(); ++i) {
            double area = cv::contourArea(contours[i]);
            if (area > max_area) { max_area = area; max_index = i; }
        }
        if (max_area > min_area_) {
            const cv::Moments moments = cv::moments(contours[max_index]);
            double x = moments.m10 / moments.m00, y = moments.m01 / moments.m00;
            geometry_msgs::Vector3Stamped target;
            target.header = msg->header;  // 源采集时间不可替换成检测/重发时间。
            target.vector.x = x / image->image.cols;
            target.vector.y = y / image->image.rows;
            target.vector.z = moments.m00 / (image->image.cols * image->image.rows);
            target_pub_.publish(target);
            geometry_msgs::Point legacy;
            legacy.x = target.vector.x; legacy.y = target.vector.y; legacy.z = target.vector.z;
            legacy_pub_.publish(legacy);
            cv::rectangle(image->image, cv::boundingRect(contours[max_index]), cv::Scalar(0, 255, 0), 2);
            cv::circle(image->image, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), -1);
        }
        // 保持原绿框/红质心调试图及源图像 header；压缩由 image_transport 提供。
        debug_pub_.publish(image->toImageMsg());
    }
private:
    ros::NodeHandle nh_, private_nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher legacy_pub_, target_pub_;
    image_transport::Publisher debug_pub_;
    cv::Mat kernel_;
    int low_h_, high_h_, low_s_, high_s_, low_v_, high_v_, min_area_, morph_kernel_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "blue_detector_node");
    try { BlueDetector detector; ros::spin(); }
    catch (const std::exception& error) { ROS_FATAL("%s", error.what()); return 1; }
    return 0;
}
