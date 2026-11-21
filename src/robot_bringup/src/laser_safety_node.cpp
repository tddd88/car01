#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <algorithm>
#include <cmath>
#include <vector>

struct Obstacle { double x,y; };

// 所有控制模式的唯一最终速度出口：带时效的输入、真实 TF、矩形扫掠和制动距离。
class LaserSafetyNode {
 public:
  LaserSafetyNode() : pnh_("~"), listener_(ros::Duration(3.0)) {
    pnh_.param<std::string>("cmd_vel_in",in_,"/cmd_vel_requested");
    pnh_.param<std::string>("cmd_vel_out",out_,"/cmd_vel");
    pnh_.param<std::string>("scan_topic",scan_topic_,"/scan");
    pnh_.param<std::string>("base_frame",base_,"base_link");
    pnh_.param("footprint_half_length",length_,0.175); pnh_.param("footprint_half_width",width_,0.11);
    pnh_.param("safety_margin",margin_,0.025); pnh_.param("reaction_time",reaction_,0.25);
    pnh_.param("linear_deceleration",deceleration_,0.6); pnh_.param("angular_deceleration",angular_deceleration_,1.2);
    pnh_.param("max_accel_x",ax_,0.4); pnh_.param("max_accel_y",ay_,0.4); pnh_.param("max_accel_yaw",aw_,0.8);
    pnh_.param("max_vx",max_vx_,0.8); pnh_.param("max_vy",max_vy_,0.5); pnh_.param("max_wz",max_wz_,0.6);
    pnh_.param("scan_timeout",scan_timeout_,0.4); pnh_.param("cmd_timeout",cmd_timeout_,0.2);
    pnh_.param("min_scan_coverage",min_coverage_,5.5); pnh_.param("min_valid_fraction",valid_fraction_,0.2);
    // 制动能力和尺寸必须有效，拒绝以坏参数计算出无限/空扫掠。
    params_ok_=std::isfinite(length_) && std::isfinite(width_) && length_>0 && width_>0 &&
      std::isfinite(margin_) && margin_>=0 && reaction_>=0 && std::isfinite(reaction_) &&
      deceleration_>=0.05 && std::isfinite(deceleration_) && angular_deceleration_>=0.1 && std::isfinite(angular_deceleration_) &&
      reaction_<=2.0 && ax_>0 && ay_>0 && aw_>0 && std::isfinite(ax_) && std::isfinite(ay_) && std::isfinite(aw_) &&
      max_vx_>0 && max_vx_<=2 && max_vy_>0 && max_vy_<=2 && max_wz_>0 && max_wz_<=3 &&
      scan_timeout_>0 && scan_timeout_<=2 && cmd_timeout_>0 && cmd_timeout_<=2 &&
      min_coverage_>0 && min_coverage_<=6.4 && valid_fraction_>0 && valid_fraction_<=1;
    scan_sub_=nh_.subscribe(scan_topic_,1,&LaserSafetyNode::scanCb,this);
    cmd_sub_=nh_.subscribe(in_,1,&LaserSafetyNode::cmdCb,this);
    pub_=nh_.advertise<geometry_msgs::Twist>(out_,1);
    ready_pub_=nh_.advertise<std_msgs::Bool>("/safety/ready",1);
    reason_pub_=nh_.advertise<std_msgs::String>("/safety/reason",1);
    previous_tick_=ros::WallTime::now();
    timer_=nh_.createWallTimer(ros::WallDuration(1.0/30.0),&LaserSafetyNode::tick,this);
  }
 private:
  ros::NodeHandle nh_,pnh_; tf::TransformListener listener_;
  ros::Subscriber scan_sub_,cmd_sub_; ros::Publisher pub_,ready_pub_,reason_pub_; ros::WallTimer timer_;
  std::string in_,out_,scan_topic_,base_,scan_error_="尚无有效雷达";
  double length_,width_,margin_,reaction_,deceleration_,angular_deceleration_,ax_,ay_,aw_;
  double max_vx_,max_vy_,max_wz_,scan_timeout_,cmd_timeout_,min_coverage_,valid_fraction_;
  bool scan_valid_=false,cmd_valid_=false,params_ok_=false,published_=false;
  ros::WallTime scan_received_,cmd_received_,previous_tick_; ros::Time scan_stamp_;
  geometry_msgs::TwistStamped command_; geometry_msgs::Twist previous_;
  std::vector<Obstacle> obstacles_;
  bool fresh(const ros::Time& s,double limit) const {
    const double age=(ros::Time::now()-s).toSec();
    return !s.isZero() && age>=-0.05 && age<=limit;
  }
  bool finite(const geometry_msgs::Twist& t) const {
    return std::isfinite(t.linear.x) && std::isfinite(t.linear.y) && std::isfinite(t.linear.z) &&
      std::isfinite(t.angular.x) && std::isfinite(t.angular.y) && std::isfinite(t.angular.z);
  }
  void cmdCb(const geometry_msgs::TwistStamped::ConstPtr& c) {
    command_=*c; cmd_received_=ros::WallTime::now();
    cmd_valid_=finite(c->twist) && fresh(c->header.stamp,cmd_timeout_) &&
      (c->header.frame_id.empty() || c->header.frame_id==base_);
  }
  void scanCb(const sensor_msgs::LaserScan::ConstPtr& s) {
    scan_valid_=false;
    if(!fresh(s->header.stamp,scan_timeout_) || s->header.frame_id.empty()) { scan_error_="雷达时间戳过期或 frame 缺失"; return; }
    if(s->ranges.size()<16 || !std::isfinite(s->angle_min) || !std::isfinite(s->angle_increment) ||
       s->angle_increment==0 || !std::isfinite(s->range_min) || !std::isfinite(s->range_max) ||
       s->range_min<0 || s->range_max<=s->range_min ||
       std::abs(s->angle_increment)*(s->ranges.size()-1)<min_coverage_) {
      scan_error_="雷达范围或覆盖角无效"; return;
    }
    tf::StampedTransform transform;
    try { listener_.lookupTransform(base_,s->header.frame_id,s->header.stamp,transform); }
    catch(const tf::TransformException& e) { scan_error_="雷达 TF 缺失或过期"; ROS_WARN_THROTTLE(2,"%s",e.what()); return; }
    std::vector<Obstacle> points; points.reserve(s->ranges.size()); size_t valid=0;
    for(size_t i=0;i<s->ranges.size();++i) {
      const double r=s->ranges[i];
      if(std::isinf(r) && r>0) { ++valid; continue; }
      if(!std::isfinite(r) || r<s->range_min || r>s->range_max) continue;
      ++valid;
      const double angle=s->angle_min+s->angle_increment*i;
      const tf::Vector3 p=transform*tf::Vector3(r*std::cos(angle),r*std::sin(angle),0);
      if(std::isfinite(p.x()) && std::isfinite(p.y())) points.push_back({p.x(),p.y()});
    }
    if(static_cast<double>(valid)/s->ranges.size()<valid_fraction_) { scan_error_="雷达有效数据比例不足"; return; }
    obstacles_.swap(points); scan_stamp_=s->header.stamp; scan_received_=ros::WallTime::now(); scan_valid_=true;
  }
  double clearance(const Obstacle& p,double x,double y,double c,double s) const {
    const double dx=p.x-x,dy=p.y-y;
    return std::max(std::abs(c*dx+s*dy)-length_,std::abs(-s*dx+c*dy)-width_);
  }
  // 允许贴近边界后纯平移离开；旋转或让任何近点更深进入车体则拒绝。
  bool escape(const Obstacle& p,const geometry_msgs::Twist& cmd) const {
    if(std::abs(cmd.angular.z)>1e-6) return false;
    const double ex=std::abs(p.x)-length_,ey=std::abs(p.y)-width_;
    if(ex>=ey) return p.x*cmd.linear.x<=0 && (std::abs(p.x)>length_ || p.y*cmd.linear.y<=0);
    return p.y*cmd.linear.y<=0 && (std::abs(p.y)>width_ || p.x*cmd.linear.x<=0);
  }
  bool trajectorySafe(const geometry_msgs::Twist& cmd) const {
    const double speed=std::hypot(cmd.linear.x,cmd.linear.y),omega=std::abs(cmd.angular.z);
    if(speed+omega<1e-8) return true;
    const double braking=std::max(speed/deceleration_,omega/angular_deceleration_);
    const double horizon=reaction_+braking,radius=std::hypot(length_,width_);
    const int steps=std::max(1,static_cast<int>(std::ceil(horizon/std::min(0.04,0.01/(speed+radius*omega)))));
    const double dt=horizon/steps;
    // 路径采样之间用最大平移+角运动的 Lipschitz 上界膨胀车体。
    const double gap=(speed+radius*omega)*dt*0.5;
    struct CandidatePoint { const Obstacle* point; double initial; bool can_escape; };
    std::vector<CandidatePoint> nearby; nearby.reserve(obstacles_.size());
    const double reach=std::hypot(length_+margin_+gap,width_+margin_+gap)+speed*horizon;
    for(const Obstacle& p:obstacles_) {
      if(p.x*p.x+p.y*p.y>reach*reach) continue;
      const double initial=clearance(p,0,0,1,0);
      nearby.push_back({&p,initial,initial<=margin_ && escape(p,cmd)});
    }
    double x=0,y=0,yaw=0;
    for(int k=0;k<=steps;++k) {
      const double c=std::cos(yaw),s=std::sin(yaw);
      for(const CandidatePoint& p:nearby) {
        const double distance=clearance(*p.point,x,y,c,s);
        if(p.initial<=margin_) {
          if(!p.can_escape || distance<p.initial-1e-6) return false;
        } else if(distance<=margin_+gap) return false;
      }
      if(k==steps) break;
      const double t=(k+0.5)*dt;
      const double factor=t<=reaction_ ? 1.0 : std::max(0.0,1.0-(t-reaction_)/std::max(1e-9,braking));
      const double mid=yaw+0.5*cmd.angular.z*factor*dt;
      x+=(std::cos(mid)*cmd.linear.x-std::sin(mid)*cmd.linear.y)*factor*dt;
      y+=(std::sin(mid)*cmd.linear.x+std::cos(mid)*cmd.linear.y)*factor*dt;
      yaw+=cmd.angular.z*factor*dt;
    }
    return true;
  }
  double ramp(double desired,double prev,double acceleration,double dt) const {
    if(desired*prev<0) return 0; // 反向先停，禁止旧方向继续滑向障碍。
    if(std::abs(desired)<=std::abs(prev)) return desired;
    return std::copysign(std::min(std::abs(desired),std::abs(prev)+acceleration*dt),desired);
  }
  static double limit(double value,double maximum) { return std::max(-maximum,std::min(maximum,value)); }
  static geometry_msgs::Twist scaled(const geometry_msgs::Twist& c,double scale) {
    geometry_msgs::Twist t; t.linear.x=c.linear.x*scale; t.linear.y=c.linear.y*scale; t.angular.z=c.angular.z*scale; return t;
  }
  void tick(const ros::WallTimerEvent&) {
    const ros::WallTime now=ros::WallTime::now();
    const double dt=std::max(0.0,std::min(0.1,(now-previous_tick_).toSec())); previous_tick_=now;
    const bool scan_ok=params_ok_ && scan_valid_ && fresh(scan_stamp_,scan_timeout_) && (now-scan_received_).toSec()<=scan_timeout_;
    const bool cmd_ok=cmd_valid_ && fresh(command_.header.stamp,cmd_timeout_) && (now-cmd_received_).toSec()<=cmd_timeout_;
    geometry_msgs::Twist result; std::string reason;
    if(!scan_ok) reason=!params_ok_ ? "安全参数无效" : (scan_valid_ ? "雷达已过期" : scan_error_);
    else if(!cmd_ok) reason="请求速度已过期或无效";
    else {
      geometry_msgs::Twist candidate;
      candidate.linear.x=ramp(limit(command_.twist.linear.x,max_vx_),previous_.linear.x,ax_,dt);
      candidate.linear.y=ramp(limit(command_.twist.linear.y,max_vy_),previous_.linear.y,ay_,dt);
      candidate.angular.z=ramp(limit(command_.twist.angular.z,max_wz_),previous_.angular.z,aw_,dt);
      if(trajectorySafe(candidate)) { result=candidate; reason="OK"; }
      else {
        double lo=0,hi=1;
        for(int i=0;i<7;++i) { const double mid=(lo+hi)*0.5; if(trajectorySafe(scaled(candidate,mid))) lo=mid; else hi=mid; }
        if(lo>=0.02) result=scaled(candidate,lo);
        reason="车体扫掠或制动空间受限";
      }
    }
    // 故障/障碍停车立即生效，不以加速度限幅拖延停车。
    // 下发规则（与 cmd_vel_repeat / joy_repeat 一致）：
    //   非零速度 30Hz 持续发（给底盘看门狗保活）；
    //   零速只在"非零→零"（停车指令）或"从未发布过"时发一次，之后静默让权。
    const bool is_zero_now=std::abs(result.linear.x)<1e-6 && std::abs(result.linear.y)<1e-6 &&
      std::abs(result.angular.z)<1e-6;
    const bool was_zero=std::abs(previous_.linear.x)<1e-6 && std::abs(previous_.linear.y)<1e-6 &&
      std::abs(previous_.angular.z)<1e-6;
    if(!is_zero_now || !was_zero || !published_) {
      pub_.publish(result);
      published_=true;
    }
    previous_=result;
    std_msgs::Bool state; state.data=scan_ok; ready_pub_.publish(state);
    std_msgs::String detail; detail.data=reason; reason_pub_.publish(detail);
  }
};
int main(int argc,char** argv) { ros::init(argc,argv,"laser_safety_node"); LaserSafetyNode node; ros::spin(); }
