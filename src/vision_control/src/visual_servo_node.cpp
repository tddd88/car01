#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>

class VisualServo {
public:
    VisualServo() {
        target_sub_ = nh_.subscribe("/blue_target", 1, &VisualServo::targetCb, this);
        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_vision", 1);
        
        nh_.param("kp_x", kp_x_, 0.5);
        nh_.param("kp_y", kp_y_, 1.0);
        target_area_ = 0.3; // 停止阈值：占比 30%
    }

    void targetCb(const geometry_msgs::Point::ConstPtr& msg) {
        geometry_msgs::Twist vel;
        
        // 视觉对中 (麦轮横移量)
        double error_y = 0.5 - msg->x; 
        vel.linear.y = error_y * kp_y_;

        // 距离控制 (直线速度)
        if (msg->z < target_area_) {
            vel.linear.x = (target_area_ - msg->z) * kp_x_;
        } else {
            vel.linear.x = 0;
            ROS_INFO_THROTTLE(1, "Target reached! Stopping.");
        }
        
        vel_pub_.publish(vel);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber target_sub_;
    ros::Publisher vel_pub_;
    double kp_x_, kp_y_, target_area_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "visual_servo_node");
    VisualServo vs;
    ros::spin();
    return 0;
}
