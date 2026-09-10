// laser_safety_node.cpp
// 激光雷达全向安全避障节点（遥控时也防撞）
// 订阅 cmd_vel_raw + /scan，按四象限最小距离抑制朝向障碍的速度分量，输出 cmd_vel
//
// 原理：
//   将 360° 扫描按机器人前/后/左/右四个象限分组，取每象限最小距离。
//   若某象限最小距离 < 安全阈值，则朝该方向的速度分量被抑制（线性减速带）。
//   前后安全距离 0.20m，左右安全距离 0.15m（可参数配置）。
//
// 话题链路：
//   joy_repeat_node -> cmd_vel_raw -> [laser_safety_node] -> cmd_vel -> stm32_bridge

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <algorithm>
#include <cmath>

class LaserSafetyNode
{
public:
    LaserSafetyNode() : nh_(), nh_priv_("~"), scan_received_(false), cmd_received_(false)
    {
        // 参数（私有命名空间读取）
        nh_priv_.param<std::string>("cmd_vel_in",  cmd_vel_in_topic_,  "cmd_vel_raw");
        nh_priv_.param<std::string>("cmd_vel_out", cmd_vel_out_topic_, "cmd_vel");
        nh_priv_.param<std::string>("scan_topic",  scan_topic_,        "/scan");

        nh_priv_.param<double>("safe_dist_front", safe_dist_front_, 0.20);
        nh_priv_.param<double>("safe_dist_rear",  safe_dist_rear_,  0.20);
        nh_priv_.param<double>("safe_dist_left",  safe_dist_left_,  0.15);
        nh_priv_.param<double>("safe_dist_right", safe_dist_right_, 0.15);
        nh_priv_.param<double>("slow_factor",    slow_factor_,     2.0);  // 减速带 = safe * slow_factor

        // 订阅 / 发布（全局命名空间，避免话题被加上节点私有前缀）
        scan_sub_  = nh_.subscribe(scan_topic_,        10, &LaserSafetyNode::scanCallback, this);
        cmd_sub_   = nh_.subscribe(cmd_vel_in_topic_,  10, &LaserSafetyNode::cmdCallback,  this);
        cmd_pub_   = nh_.advertise<geometry_msgs::Twist>(cmd_vel_out_topic_, 10);
    }

    void spin()
    {
        ros::Rate rate(30);  // 30Hz 输出
        ros::Duration cmd_timeout(0.5);  // 0.5 秒无输入则停止转发，让权给其他 cmd_vel 源

        while (ros::ok())
        {
            // 超时检查：若超过 cmd_timeout 未收到 cmd_vel_raw，停止转发
            // 避免 joy_repeat_node 停发后，本节点继续用旧 last_cmd_ 持续刷 cmd_vel
            if (cmd_received_ && (ros::Time::now() - last_cmd_time_ > cmd_timeout))
            {
                cmd_received_ = false;
                last_cmd_ = geometry_msgs::Twist();  // 清零
            }

            // 无输入速度时不发，避免抢占其他 cmd_vel 源（如决策状态机）
            if (cmd_received_)
            {
                geometry_msgs::Twist out = last_cmd_;
                if (scan_received_)
                {
                    applySafety(out);
                }
                cmd_pub_.publish(out);
            }
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;       // 全局命名空间（用于话题订阅/发布）
    ros::NodeHandle nh_priv_;  // 私有命名空间（用于读参数）
    ros::Subscriber scan_sub_;
    ros::Subscriber cmd_sub_;
    ros::Publisher  cmd_pub_;

    std::string cmd_vel_in_topic_;
    std::string cmd_vel_out_topic_;
    std::string scan_topic_;

    double safe_dist_front_, safe_dist_rear_, safe_dist_left_, safe_dist_right_;
    double slow_factor_;

    sensor_msgs::LaserScan last_scan_;
    geometry_msgs::Twist    last_cmd_;
    ros::Time               last_cmd_time_;
    bool scan_received_;
    bool cmd_received_;

    // 四象限最小距离
    double min_front_, min_rear_, min_left_, min_right_;

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
    {
        last_scan_ = *msg;
        scan_received_ = true;
        computeMinDistances();
    }

    void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        last_cmd_ = *msg;
        last_cmd_time_ = ros::Time::now();
        cmd_received_ = true;
    }

    // 将扫描角度按四象限分组，取每象限最小有效距离
    void computeMinDistances()
    {
        const sensor_msgs::LaserScan& s = last_scan_;
        min_front_ = min_rear_ = min_left_ = min_right_ = std::numeric_limits<double>::infinity();

        double angle = s.angle_min;
        double a_inc = s.angle_increment;
        for (size_t i = 0; i < s.ranges.size(); ++i, angle += a_inc)
        {
            float r = s.ranges[i];
            // 过滤无效点
            if (r < s.range_min || r > s.range_max || !std::isfinite(r)) continue;

            // 归一化角度到 [-pi, pi]
            double a = angle;
            while (a >  M_PI) a -= 2.0 * M_PI;
            while (a < -M_PI) a += 2.0 * M_PI;

            // 四象限划分（机器人坐标系：x 前、y 左）
            // 前: [-pi/4,  pi/4]
            // 左: [ pi/4,  3pi/4]
            // 后: [3pi/4, pi] ∪ [-pi, -3pi/4]
            // 右: [-3pi/4, -pi/4]
            if (a >= -M_PI/4 && a < M_PI/4)
            {
                if (r < min_front_) min_front_ = r;
            }
            else if (a >= M_PI/4 && a < 3.0*M_PI/4)
            {
                if (r < min_left_) min_left_ = r;
            }
            else if (a >= 3.0*M_PI/4 || a < -3.0*M_PI/4)
            {
                if (r < min_rear_) min_rear_ = r;
            }
            else  // [-3pi/4, -pi/4)
            {
                if (r < min_right_) min_right_ = r;
            }
        }
    }

    // 线性减速：距离在 [safe, safe*slow_factor] 之间线性缩放速度到 [0, 1]
    // 距离 < safe 时完全抑制；距离 > safe*slow_factor 时不限制
    double scaleFor(double dist, double safe)
    {
        if (!std::isfinite(dist)) return 1.0;          // 无障碍
        if (dist <= safe) return 0.0;                   // 太近，完全抑制
        double far = safe * slow_factor_;
        if (dist >= far) return 1.0;                    // 足够远，不限制
        return (dist - safe) / (far - safe);            // 线性减速带
    }

    void applySafety(geometry_msgs::Twist& cmd)
    {
        // 前后（x 方向）
        double sx = 1.0;
        if (cmd.linear.x > 0)
            sx = scaleFor(min_front_, safe_dist_front_);
        else if (cmd.linear.x < 0)
            sx = scaleFor(min_rear_, safe_dist_rear_);
        cmd.linear.x *= sx;

        // 左右（y 方向）
        double sy = 1.0;
        if (cmd.linear.y > 0)
            sy = scaleFor(min_left_, safe_dist_left_);
        else if (cmd.linear.y < 0)
            sy = scaleFor(min_right_, safe_dist_right_);
        cmd.linear.y *= sy;

        // 旋转不限制（原地转不撞墙）
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_safety_node");
    LaserSafetyNode node;
    node.spin();
    return 0;
}
