#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

// 持续发送版手柄控制节点 (麦轮全向移动版)
// 替代 teleop_twist_joy (0.1.3 不支持 autorepeat_rate)
// 只要使能键按住，就按固定频率持续下发 cmd_vel
//
// 映射: 左摇杆前后 -> vx, 左摇杆左右 -> vy, 右摇杆左右 -> vth

static geometry_msgs::Twist g_last_cmd;
static bool g_enabled = false;
static bool g_joy_received = false;

bool hasVelocity(const geometry_msgs::Twist& cmd) {
    return cmd.linear.x != 0.0 || cmd.linear.y != 0.0 || cmd.linear.z != 0.0 ||
           cmd.angular.x != 0.0 || cmd.angular.y != 0.0 || cmd.angular.z != 0.0;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {
    g_joy_received = true;

    // 参数
    static ros::NodeHandle nh_priv("~");
    static int enable_button   = nh_priv.param<int>("enable_button", 7);
    static int axis_vx         = nh_priv.param<int>("axis_vx", 1);   // 左摇杆前后
    static int axis_vy         = nh_priv.param<int>("axis_vy", 0);   // 左摇杆左右
    static int axis_vth        = nh_priv.param<int>("axis_vth", 2);   // 右摇杆左右
    static double scale_vx     = nh_priv.param<double>("scale_vx", 0.5);
    static double scale_vy     = nh_priv.param<double>("scale_vy", 0.5);
    static double scale_vth    = nh_priv.param<double>("scale_vth", 0.5);

    if (enable_button >= 0 && enable_button < (int)joy->buttons.size()) {
        g_enabled = joy->buttons[enable_button] == 1;
    }

    if (g_enabled) {
        double vx = 0.0, vy = 0.0, vth = 0.0;
        if (axis_vx >= 0 && axis_vx < (int)joy->axes.size())
            vx = joy->axes[axis_vx];
        if (axis_vy >= 0 && axis_vy < (int)joy->axes.size())
            vy = joy->axes[axis_vy];
        if (axis_vth >= 0 && axis_vth < (int)joy->axes.size())
            vth = joy->axes[axis_vth];

        g_last_cmd.linear.x  = vx  * scale_vx;
        g_last_cmd.linear.y  = vy  * scale_vy;
        g_last_cmd.angular.z = vth * scale_vth;
    } else {
        // 松开使能键立即停车
        g_last_cmd.linear.x  = 0.0;
        g_last_cmd.linear.y  = 0.0;
        g_last_cmd.angular.z = 0.0;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "teleop_twist_joy");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    double publish_rate = nh_priv.param<double>("autorepeat_rate", 20.0);
    if (publish_rate <= 0.0) publish_rate = 20.0;

    ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, joyCallback);
    ros::Publisher cmd_pub  = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    ros::Rate rate(publish_rate);
    bool was_moving = false;

    while (ros::ok()) {
        ros::spinOnce();

        bool moving = g_joy_received && hasVelocity(g_last_cmd);
        // 非零速度持续发布；由非零变为零时只发布一次停车指令
        if (moving || (was_moving && g_joy_received)) {
            cmd_pub.publish(g_last_cmd);
        }
        was_moving = moving;
        rate.sleep();
    }

    return 0;
}
