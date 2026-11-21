#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <algorithm>

// 只重放新鲜的真实 Joy；速度重复消息不能作为手柄仍在线的证据。
class JoyRepeatNode {
 public:
  JoyRepeatNode() : pnh_("~") {
    pnh_.param("enable_button", enable_button_, 5);
    pnh_.param("axis_vx", ax_, 1); pnh_.param("axis_vy", ay_, 0);
    pnh_.param("axis_vth", aw_, 3);
    pnh_.param("scale_vx", sx_, 0.5); pnh_.param("scale_vy", sy_, 0.5);
    pnh_.param("scale_vth", sw_, 0.5); pnh_.param("joy_timeout", timeout_, 0.5);
    pnh_.param("deadband", deadband_, 0.05);
    double rate; pnh_.param("autorepeat_rate", rate, 20.0);
    sub_ = nh_.subscribe("joy", 1, &JoyRepeatNode::joyCb, this);
    pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    timer_ = nh_.createWallTimer(ros::WallDuration(1.0/std::max(20.0,rate)), &JoyRepeatNode::tick, this);
  }
 private:
  ros::NodeHandle nh_, pnh_; ros::Subscriber sub_; ros::Publisher pub_; ros::WallTimer timer_;
  int enable_button_, ax_, ay_, aw_; double sx_, sy_, sw_, timeout_, deadband_;
  ros::WallTime received_; ros::Time stamp_; bool valid_ = false;
  geometry_msgs::Twist cmd_;
  double axis(const sensor_msgs::Joy& j, int index) {
    if(index<0 || index>=static_cast<int>(j.axes.size()) || !std::isfinite(j.axes[index])) return 0.0;
    const double a=std::max(-1.0,std::min(1.0,static_cast<double>(j.axes[index])));
    return std::abs(a)<deadband_ ? 0.0 : a;
  }
  void joyCb(const sensor_msgs::Joy::ConstPtr& j) {
    cmd_=geometry_msgs::Twist(); valid_=false;
    const double age=(ros::Time::now()-j->header.stamp).toSec();
    if(j->header.stamp.isZero() || age < -0.05 || age>timeout_ || enable_button_<0 ||
       enable_button_>=static_cast<int>(j->buttons.size())) return;
    for(float a:j->axes) if(!std::isfinite(a)) return;
    received_=ros::WallTime::now(); stamp_=j->header.stamp; valid_=true;
    if(j->buttons[enable_button_]) {
      cmd_.linear.x=axis(*j,ax_)*sx_; cmd_.linear.y=axis(*j,ay_)*sy_;
      cmd_.angular.z=axis(*j,aw_)*sw_;
    }
  }
  void tick(const ros::WallTimerEvent&) {
    const double age=(ros::Time::now()-stamp_).toSec();
    const bool fresh=valid_ && (ros::WallTime::now()-received_).toSec()<=timeout_ && age>=-0.05 && age<=timeout_;
    pub_.publish(fresh ? cmd_ : geometry_msgs::Twist());
  }
};
int main(int argc,char** argv) { ros::init(argc,argv,"teleop_twist_joy"); JoyRepeatNode node; ros::spin(); }
