#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <cmath>

// 人工优先/失联锁止由真实 Joy 判定；自动和人工共同进入唯一安全出口。
class CmdVelArbiter {
 public:
  CmdVelArbiter() : pnh_("~") {
    pnh_.param("enable_button",enable_button_,5); pnh_.param("joy_timeout",joy_timeout_,0.5);
    pnh_.param("raw_timeout",raw_timeout_,0.2); pnh_.param("auto_timeout",auto_timeout_,0.3);
    pnh_.param<std::string>("base_frame",base_frame_,"base_link");
    joy_sub_=nh_.subscribe("/joy",1,&CmdVelArbiter::joyCb,this);
    raw_sub_=nh_.subscribe("/cmd_vel_raw",1,&CmdVelArbiter::rawCb,this);
    auto_sub_=nh_.subscribe("/cmd_vel_auto",1,&CmdVelArbiter::autoCb,this);
    pub_=nh_.advertise<geometry_msgs::TwistStamped>("/cmd_vel_requested",1);
    manual_pub_=nh_.advertise<std_msgs::Bool>("/control/manual_active",1);
    ready_pub_=nh_.advertise<std_msgs::Bool>("/control/ready",1);
    reason_pub_=nh_.advertise<std_msgs::String>("/control/reason",1);
    reset_=nh_.advertiseService("/control/reset",&CmdVelArbiter::resetCb,this);
    stop_=nh_.advertiseService("/control/stop",&CmdVelArbiter::stopCb,this);
    timer_=nh_.createWallTimer(ros::WallDuration(0.04),&CmdVelArbiter::tick,this);
  }
 private:
  ros::NodeHandle nh_,pnh_; ros::Subscriber joy_sub_,raw_sub_,auto_sub_;
  ros::Publisher pub_,manual_pub_,ready_pub_,reason_pub_; ros::ServiceServer reset_,stop_; ros::WallTimer timer_;
  int enable_button_; double joy_timeout_,raw_timeout_,auto_timeout_; std::string base_frame_;
  bool joy_valid_=false, enabled_=false, manual_used_=false, lock_=false,raw_valid_=false,auto_valid_=false;
  bool operator_stop_=false;
  bool last_published_zero_=true;  // 上一帧实际发布的是否为零速（初始视为零，确保启动后第一帧必发）
  bool just_published_=false;      // 上一 tick 是否真的发布了消息
  ros::WallTime joy_time_,raw_time_,auto_time_; ros::Time joy_stamp_,release_stamp_;
  geometry_msgs::Twist raw_; geometry_msgs::TwistStamped automatic_;
  bool stampFresh(const ros::Time& stamp,double timeout) const {
    const double age=(ros::Time::now()-stamp).toSec();
    return !stamp.isZero() && age>=-0.05 && age<=timeout;
  }
  bool joyFresh() const { return joy_valid_ && stampFresh(joy_stamp_,joy_timeout_) &&
    (ros::WallTime::now()-joy_time_).toSec()<=joy_timeout_; }
  bool finite(const geometry_msgs::Twist& t) const {
    return std::isfinite(t.linear.x) && std::isfinite(t.linear.y) && std::isfinite(t.linear.z) &&
      std::isfinite(t.angular.x) && std::isfinite(t.angular.y) && std::isfinite(t.angular.z);
  }
  bool zero(const geometry_msgs::Twist& t) const { return std::abs(t.linear.x)<1e-6 &&
    std::abs(t.linear.y)<1e-6 && std::abs(t.angular.z)<1e-6; }
  // 摇杆是否全部归中（用于安全地自动解除失联锁止）
  bool axesCentered(const sensor_msgs::Joy& j) const {
    for(float a:j.axes) if(std::isfinite(a) && std::abs(a)>0.15f) return false;
    return true;
  }
  void joyCb(const sensor_msgs::Joy::ConstPtr& j) {
    joy_valid_=false;
    if(!stampFresh(j->header.stamp,joy_timeout_) || enable_button_<0 ||
       enable_button_>=static_cast<int>(j->buttons.size())) { if(manual_used_) lock_=true; return; }
    for(float a:j->axes) if(!std::isfinite(a)) { lock_=true; return; }
    joy_stamp_=j->header.stamp; joy_time_=ros::WallTime::now(); joy_valid_=true;
    const bool was_enabled=enabled_; enabled_=j->buttons[enable_button_]!=0;
    // 自动解除"因 /joy 失联(xpad/USB 掉线等)产生的锁止"：
    // 手柄恢复且使能键已松开、摇杆全部归中时，等价于 /control/reset 的手工前置条件。
    // 人工 /control/stop 的锁止(operator_stop_)不自动解除，必须显式 reset。
    if(lock_ && !operator_stop_ && !enabled_ && axesCentered(*j)) {
      lock_=false; release_stamp_=ros::Time::now(); auto_valid_=false;
      ROS_WARN_THROTTLE(2.0,"cmd_vel_arbiter: 手柄已恢复，自动解除失联锁止");
    }
    if(enabled_) { manual_used_=true; auto_valid_=false; }
    else if(was_enabled) { release_stamp_=ros::Time::now(); auto_valid_=false; }
  }
  void rawCb(const geometry_msgs::Twist::ConstPtr& t) {
    raw_=*t; raw_valid_=finite(raw_); raw_time_=ros::WallTime::now();
  }
  void autoCb(const geometry_msgs::TwistStamped::ConstPtr& t) {
    automatic_=*t; auto_time_=ros::WallTime::now();
    auto_valid_=!enabled_ && !lock_ && !operator_stop_ && finite(t->twist) && stampFresh(t->header.stamp,auto_timeout_) &&
      (release_stamp_.isZero() || t->header.stamp>=release_stamp_) &&
      (t->header.frame_id.empty() || t->header.frame_id==base_frame_);
  }
  bool resetCb(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& r) {
    if(!joyFresh() || enabled_ || (raw_valid_ && !zero(raw_))) {
      r.success=false; r.message="需新鲜手柄输入、释放使能并归零后才能解除锁止"; return true;
    }
    lock_=false; operator_stop_=false; automatic_=geometry_msgs::TwistStamped(); auto_valid_=false;
    release_stamp_=ros::Time::now();
    r.success=true; r.message="控制锁止已解除；自动比赛仍需显式恢复"; return true;
  }
  bool stopCb(std_srvs::Trigger::Request&,std_srvs::Trigger::Response& r) {
    operator_stop_=true; auto_valid_=false;
    geometry_msgs::TwistStamped stopped; stopped.header.stamp=ros::Time::now(); stopped.header.frame_id=base_frame_;
    pub_.publish(stopped);
    std_msgs::Bool state; state.data=false; ready_pub_.publish(state);
    state.data=true; manual_pub_.publish(state);
    std_msgs::String reason; reason.data="OPERATOR_STOP"; reason_pub_.publish(reason);
    r.success=true; r.message="控制已显式停止并锁止，需释放使能/归零后 control/reset"; return true;
  }
  void tick(const ros::WallTimerEvent&) {
    const ros::WallTime now=ros::WallTime::now(); const bool fresh=joyFresh();
    if(manual_used_ && !fresh) lock_=true;
    const bool manual=(fresh && enabled_) || lock_ || operator_stop_;
    geometry_msgs::TwistStamped out; out.header.stamp=ros::Time::now(); out.header.frame_id=base_frame_;
    if(!lock_ && !operator_stop_) {
      if(manual && raw_valid_ && (now-raw_time_).toSec()<=raw_timeout_) out.twist=raw_;
      else if(!manual && auto_valid_ && stampFresh(automatic_.header.stamp,auto_timeout_) &&
              (now-auto_time_).toSec()<=auto_timeout_) out.twist=automatic_.twist;
    }
    // 底盘只消费平面速度；未知分量不能穿过安全层。
    out.twist.linear.z=0; out.twist.angular.x=0; out.twist.angular.y=0;
    // 下发规则（与 cmd_vel_repeat_node/joy_repeat 一致）：
    //   非零速度 25Hz 持续发；零速只在"非零->零"或"上一帧为静默"时发一次，之后静默。
    //   失联/锁止后恢复发布时必须先发一帧零，确保底盘不残留旧速度。
    const bool is_zero_now=std::abs(out.twist.linear.x)<1e-6 &&
      std::abs(out.twist.linear.y)<1e-6 && std::abs(out.twist.angular.z)<1e-6;
    const bool was_silent=last_published_zero_ && !just_published_;
    if(!is_zero_now || !last_published_zero_ || was_silent) {
      pub_.publish(out);
      last_published_zero_=is_zero_now;
      just_published_=true;
    } else {
      just_published_=false;
    }
    std_msgs::Bool state; state.data=manual; manual_pub_.publish(state);
    state.data=!lock_ && !operator_stop_; ready_pub_.publish(state);
    std_msgs::String reason; reason.data=operator_stop_ ? "OPERATOR_STOP" : (lock_ ? "INPUT_LOST" : (manual ? "MANUAL" : "READY"));
    reason_pub_.publish(reason);
  }
};
int main(int argc,char** argv) { ros::init(argc,argv,"cmd_vel_arbiter_node"); CmdVelArbiter node; ros::spin(); }
