// chassis_corrector_node.cpp
// 麦轮横移漂移补偿节点
//
// 原理：
//   麦轮横移时对角轮打滑不一致，产生寄生纵向速度(vx)和寄生偏航(vth)。
//   本节点用 STM32 编码器实测的 /odom_raw 反馈，对 vx 和 vth 做闭环补偿。
//   vy（横移指令本身）不补偿，避免正反馈震荡。
//
// 链路：
//   joy_repeat_node / move_base / visual_servo -> cmd_vel_raw -> [chassis_corrector] -> cmd_vel -> stm32_bridge
//                                                                          ↑
//                                                                   /odom_raw (编码器反馈)
//
// 下发规则（与 joy_repeat_node 一致）：
//   - 非零速度：按 publish_rate 持续发送
//   - 零速度：只发送一次停车指令，然后静默（不抢占其他 cmd_vel 源）

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <algorithm>
#include <cmath>

class ChassisCorrector
{
public:
    ChassisCorrector() : nh_(), nh_priv_("~"),
                         cmd_received_(false), odom_received_(false),
                         was_moving_(false), last_cmd_time_(0.0)
    {
        // ---- 话题参数 ----
        nh_priv_.param<std::string>("cmd_vel_in",  cmd_vel_in_,  "cmd_vel_raw");
        nh_priv_.param<std::string>("cmd_vel_out", cmd_vel_out_, "cmd_vel");
        nh_priv_.param<std::string>("odom_topic",  odom_topic_,  "odom_raw");

        // ---- 补偿增益 ----
        nh_priv_.param<double>("kp_vx",  kp_vx_,  0.8);   // 前后漂移补偿增益
        nh_priv_.param<double>("kp_vth", kp_vth_, 0.8);  // 偏航补偿增益

        // ---- 死区（误差小于此值不补偿，避免静止抖动）----
        nh_priv_.param<double>("deadband_vx",  deadband_vx_,  0.02);  // m/s
        nh_priv_.param<double>("deadband_vth", deadband_vth_, 0.05); // rad/s

        // ---- 低通滤波系数（0~1，越大越平滑）----
        nh_priv_.param<double>("filter_alpha", filter_alpha_, 0.3);

        // ---- 横移激活阈值：vy 绝对值超过此值才启用补偿 ----
        nh_priv_.param<double>("strafe_activate", strafe_activate_, 0.05); // m/s

        // ---- 发布频率 ----
        nh_priv_.param<double>("publish_rate", publish_rate_, 20.0);

        // ---- 输入超时：超过此时间无 cmd_vel_raw 则停止转发 ----
        nh_priv_.param<double>("cmd_timeout", cmd_timeout_, 0.5);

        cmd_sub_  = nh_.subscribe(cmd_vel_in_,  10, &ChassisCorrector::cmdCallback, this);
        odom_sub_ = nh_.subscribe(odom_topic_, 10, &ChassisCorrector::odomCallback, this);
        cmd_pub_  = nh_.advertise<geometry_msgs::Twist>(cmd_vel_out_, 10);

        // 滤波后的反馈值
        filt_vx_  = 0.0;
        filt_vth_ = 0.0;
    }

    void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        last_cmd_ = *msg;
        last_cmd_time_ = ros::Time::now();
        cmd_received_ = true;
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        double raw_vx  = msg->twist.twist.linear.x;
        double raw_vth = msg->twist.twist.angular.z;

        // 一阶低通滤波
        filt_vx_  = filter_alpha_ * raw_vx  + (1.0 - filter_alpha_) * filt_vx_;
        filt_vth_ = filter_alpha_ * raw_vth + (1.0 - filter_alpha_) * filt_vth_;

        odom_received_ = true;
    }

    static bool hasVelocity(const geometry_msgs::Twist& cmd)
    {
        return std::abs(cmd.linear.x)  > 1e-6 ||
               std::abs(cmd.linear.y)  > 1e-6 ||
               std::abs(cmd.angular.z) > 1e-6;
    }

    void spin()
    {
        ros::Rate rate(publish_rate_);
        ros::Duration timeout(cmd_timeout_);

        while (ros::ok())
        {
            // 输入超时：停止转发，让权给其他 cmd_vel 源
            if (cmd_received_ && (ros::Time::now() - last_cmd_time_ > timeout))
            {
                cmd_received_ = false;
                last_cmd_ = geometry_msgs::Twist(); // 清零
            }

            if (cmd_received_)
            {
                geometry_msgs::Twist out = last_cmd_;

                // 仅在横移时启用补偿（纯前进/旋转不需要）
                bool strafing = std::abs(last_cmd_.linear.y) > strafe_activate_;

                if (strafing && odom_received_)
                {
                    // 误差 = 期望 - 实测
                    double err_vx  = last_cmd_.linear.x  - filt_vx_;
                    double err_vth = last_cmd_.angular.z - filt_vth_;

                    // 死区
                    if (std::abs(err_vx) > deadband_vx_)
                        out.linear.x  = last_cmd_.linear.x  + kp_vx_  * err_vx;
                    if (std::abs(err_vth) > deadband_vth_)
                        out.angular.z = last_cmd_.angular.z + kp_vth_ * err_vth;
                }

                // 下发规则：非零持续发，零只发一次
                bool moving = hasVelocity(out);
                if (moving || (was_moving_ && !moving))
                {
                    cmd_pub_.publish(out);
                }
                was_moving_ = moving;
            }
            else
            {
                was_moving_ = false;
            }

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_, nh_priv_;
    ros::Subscriber cmd_sub_, odom_sub_;
    ros::Publisher  cmd_pub_;

    std::string cmd_vel_in_, cmd_vel_out_, odom_topic_;

    double kp_vx_, kp_vth_;
    double deadband_vx_, deadband_vth_;
    double filter_alpha_;
    double strafe_activate_;
    double publish_rate_;
    double cmd_timeout_;

    geometry_msgs::Twist last_cmd_;
    ros::Time last_cmd_time_;
    bool cmd_received_, odom_received_;
    bool was_moving_;

    double filt_vx_, filt_vth_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "chassis_corrector_node");
    ChassisCorrector cc;
    cc.spin();
    return 0;
}
