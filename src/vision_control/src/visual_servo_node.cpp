#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

class VisualServo {
public:
    VisualServo() {
        target_sub_ = nh_.subscribe("/blue_target", 1, &VisualServo::targetCb, this);
        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_vision", 1);

        // ---- 可调 PID 参数 ----
        nh_.param("kp_x",    kp_x_,    0.5);
        nh_.param("kp_y",    kp_y_,    1.0);
        nh_.param("kp_yaw",  kp_yaw_,  1.5);
        nh_.param("target_area", target_area_, 0.30); // 蓝纸占比达此值判定贴靠到位
        nh_.param("max_vx",  max_vx_,  0.25);  // 直线接近最大速度
        nh_.param("max_vy",  max_vy_,  0.30);  // 横移最大速度
        nh_.param("max_wz",  max_wz_,  0.60);  // 偏航最大角速度
        // 目标丢失超时（秒）：超过此时间未收到 /blue_target 则发一次零速停车
        nh_.param("target_timeout", target_timeout_, 0.3);
        // 发布频率（Hz）
        nh_.param("publish_rate", publish_rate_, 20.0);
    }

    void targetCb(const geometry_msgs::Point::ConstPtr& msg) {
        last_target_ = *msg;
        last_target_time_ = ros::Time::now();
        has_target_ = true;
    }

    void spin() {
        ros::Rate rate(publish_rate_);
        bool was_moving = false;

        while (ros::ok()) {
            ros::spinOnce();

            geometry_msgs::Twist vel;
            bool moving = false;

            if (has_target_) {
                // 目标丢失超时检测
                if (ros::Time::now() - last_target_time_ > ros::Duration(target_timeout_)) {
                    has_target_ = false;
                } else {
                    // 目标归一化中心 x: 0~1，0.5 为画面正中
                    double err_x = 0.5 - last_target_.x;

                    // 距离控制（直线接近）：占比越小离得越远，前移
                    if (last_target_.z < target_area_) {
                        // 目标未到达：偏航修正 + 横向对中 + 直线接近
                        vel.angular.z = clamp(err_x * kp_yaw_, -max_wz_, max_wz_);
                        vel.linear.y = clamp(err_x * kp_y_, -max_vy_, max_vy_);
                        vel.linear.x = clamp((target_area_ - last_target_.z) * kp_x_, 0.0, max_vx_);
                    } else {
                        // 目标到达：整体停车，不再做偏航/横移修正
                        vel.linear.x = 0.0;
                        vel.linear.y = 0.0;
                        vel.angular.z = 0.0;
                        ROS_INFO_THROTTLE(1, "Target reached! Stopping.");
                    }
                    moving = hasVelocity(vel);
                }
            }

            // 非零速度持续发布；由非零变为零时只发布一次停车指令
            if (moving || (was_moving && !moving)) {
                vel_pub_.publish(vel);
            }
            was_moving = moving;

            rate.sleep();
        }
    }

private:
    static double clamp(double v, double lo, double hi) {
        return std::max(lo, std::min(hi, v));
    }

    static bool hasVelocity(const geometry_msgs::Twist& v) {
        return v.linear.x != 0.0 || v.linear.y != 0.0 || v.linear.z != 0.0 ||
               v.angular.x != 0.0 || v.angular.y != 0.0 || v.angular.z != 0.0;
    }

    ros::NodeHandle nh_;
    ros::Subscriber target_sub_;
    ros::Publisher vel_pub_;
    geometry_msgs::Point last_target_;
    ros::Time last_target_time_;
    bool has_target_ = false;
    double kp_x_, kp_y_, kp_yaw_, target_area_;
    double max_vx_, max_vy_, max_wz_;
    double target_timeout_, publish_rate_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "visual_servo_node");
    VisualServo vs;
    vs.spin();
    return 0;
}
