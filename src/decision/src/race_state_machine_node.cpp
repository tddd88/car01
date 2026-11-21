#include <actionlib/client/simple_action_client.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <string>

using MoveBaseClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;

class RaceStateMachine {
 public:
  enum class State { WAIT_READY, NAVIGATING, SEARCHING, VISUAL, PAUSED, FINISHED, FAULT };

  RaceStateMachine()
      : pnh_("~"), ac_("move_base", true), tf_listener_(tf_buffer_), state_(State::WAIT_READY) {
    pnh_.param("auto_start", auto_start_, true);
    pnh_.param("allow_visual_during_navigation", allow_visual_during_navigation_, false);
    pnh_.param("readiness_timeout", readiness_timeout_, 0.5);
    pnh_.param("tf_timeout", tf_timeout_, 0.5);
    pnh_.param("nav_cmd_timeout", nav_cmd_timeout_, 0.5);
    pnh_.param("vision_cmd_timeout", vision_cmd_timeout_, 0.3);
    pnh_.param("target_timeout", target_timeout_, 0.3);
    pnh_.param("goal_timeout", goal_timeout_, 120.0);
    pnh_.param("search_speed", search_speed_, 0.20);
    pnh_.param("search_max_time", search_max_time_, 20.0);
    pnh_.param("search_max_angle", search_max_angle_, 6.3);
    pnh_.param("search_max_attempts", search_max_attempts_, 1);
    pnh_.param("target_stable_time", target_stable_time_, 0.4);
    pnh_.param("reached_stable_time", reached_stable_time_, 0.8);
    // 进入 VISUAL 后等待视觉首帧状态的宽限期，避免"状态未就绪"被误判为目标丢失。
    pnh_.param("visual_grace_timeout", visual_grace_timeout_, 1.0);

    waypoint_sub_ = nh_.subscribe("/vision_start_point", 1, &RaceStateMachine::waypointCb, this);
    nav_vel_sub_ = nh_.subscribe("/cmd_vel_nav", 5, &RaceStateMachine::navVelCb, this);
    target_sub_ = nh_.subscribe("/vision/target", 5, &RaceStateMachine::targetCb, this);
    vision_vel_sub_ = nh_.subscribe("/cmd_vel_vision_stamped", 5, &RaceStateMachine::visionVelCb, this);
    vision_status_sub_ = nh_.subscribe("/visual_servo/status", 5, &RaceStateMachine::visionStatusCb, this);
    scan_sub_ = nh_.subscribe("/scan", 2, &RaceStateMachine::scanCb, this);
    control_ready_sub_ = nh_.subscribe("/control/ready", 5, &RaceStateMachine::controlReadyCb, this);
    manual_sub_ = nh_.subscribe("/control/manual_active", 5, &RaceStateMachine::manualCb, this);
    safety_ready_sub_ = nh_.subscribe("/safety/ready", 5, &RaceStateMachine::safetyReadyCb, this);
    localization_ready_sub_ = nh_.subscribe("/localization/ready", 5, &RaceStateMachine::localizationReadyCb, this);
    auto_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/cmd_vel_auto", 5);
    state_pub_ = nh_.advertise<std_msgs::String>("/race/state", 1, true);
    start_srv_ = nh_.advertiseService("/race/start", &RaceStateMachine::startSrv, this);
    pause_srv_ = nh_.advertiseService("/race/pause", &RaceStateMachine::pauseSrv, this);
    resume_srv_ = nh_.advertiseService("/race/resume", &RaceStateMachine::resumeSrv, this);
    reset_srv_ = nh_.advertiseService("/race/reset", &RaceStateMachine::resetSrv, this);
    publishState();
  }

  void run() {
    ros::Rate rate(20.0);
    while (ros::ok()) { ros::spinOnce(); tick(); rate.sleep(); }
  }

 private:
  static bool finitePose(const geometry_msgs::PoseStamped& p) {
    const auto& q = p.pose.orientation;
    const double n = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    return p.header.frame_id == "map" && std::isfinite(p.pose.position.x) &&
           std::isfinite(p.pose.position.y) && std::isfinite(n) && std::abs(n - 1.0) < 1e-2;
  }
  void waypointCb(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    waypoint_valid_ = finitePose(*msg);
    if (waypoint_valid_) waypoint_ = *msg;
    else ROS_ERROR("拒绝无效预备点：必须是 map 坐标、有限数值和归一化四元数");
  }
  void navVelCb(const geometry_msgs::Twist::ConstPtr& msg) { nav_vel_ = *msg; nav_vel_time_ = ros::Time::now(); }
  void targetCb(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
    if (msg->header.stamp.isZero() || !std::isfinite(msg->vector.x) || !std::isfinite(msg->vector.y) ||
        !std::isfinite(msg->vector.z) || msg->vector.x < 0 || msg->vector.x > 1 ||
        msg->vector.y < 0 || msg->vector.y > 1 || msg->vector.z < 0 || msg->vector.z > 1) return;
    if (!targetFresh()) target_first_time_ = ros::Time::now();
    target_time_ = msg->header.stamp;
  }
  void visionVelCb(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    if (!msg->header.stamp.isZero()) { vision_vel_ = *msg; vision_vel_recv_time_ = ros::Time::now(); }
  }
  void visionStatusCb(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg) {
    if (msg->status.empty() || msg->status[0].name != "visual_servo" || msg->header.stamp.isZero()) return;
    vision_status_ = msg->status[0].message;
    vision_status_stamp_ = msg->header.stamp;
    vision_status_recv_time_ = ros::Time::now();
    if (vision_status_ == "REACHED") { if (reached_since_.isZero()) reached_since_ = ros::Time::now(); }
    else reached_since_ = ros::Time();
  }
  void scanCb(const sensor_msgs::LaserScan::ConstPtr& msg) { scan_time_ = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp; }
  void controlReadyCb(const std_msgs::Bool::ConstPtr& msg) { control_ready_ = msg->data; control_ready_time_ = ros::Time::now(); }
  void manualCb(const std_msgs::Bool::ConstPtr& msg) {
    manual_active_ = msg->data; manual_time_ = ros::Time::now();
    if (msg->data) {
      manual_latched_ = true;
      pauseForManual();  // 回调当下即取消 goal，避免一次快速按放被循环采样漏掉。
    }
  }
  void safetyReadyCb(const std_msgs::Bool::ConstPtr& msg) { safety_ready_ = msg->data; safety_ready_time_ = ros::Time::now(); }
  void localizationReadyCb(const std_msgs::Bool::ConstPtr& msg) { localization_ready_ = msg->data; localization_ready_time_ = ros::Time::now(); }

  bool fresh(const ros::Time& t, double age) const { return !t.isZero() && (ros::Time::now() - t).toSec() <= age; }
  bool targetFresh() const { return fresh(target_time_, target_timeout_); }
  bool controlFreshAndReady() const {
    return fresh(control_ready_time_, readiness_timeout_) && fresh(manual_time_, readiness_timeout_) && control_ready_;
  }
  bool tfFresh(double* yaw = nullptr) {
    try {
      const auto tf = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(0.02));
      if (!fresh(tf.header.stamp, tf_timeout_)) return false;
      if (yaw) *yaw = tf2::getYaw(tf.transform.rotation);
      return true;
    } catch (const tf2::TransformException&) { return false; }
  }
  bool ready(std::string* why = nullptr) {
    std::string reason;
    if (!waypoint_valid_) reason = "缺少有效预备点";
    else if (!controlFreshAndReady()) reason = "control 心跳缺失或未就绪";
    else if (!fresh(safety_ready_time_, readiness_timeout_) || !safety_ready_) reason = "safety 心跳缺失或未就绪";
    else if (!fresh(localization_ready_time_, readiness_timeout_) || !localization_ready_) reason = "localization 心跳缺失或未就绪";
    else if (!fresh(scan_time_, readiness_timeout_)) reason = "scan 不新鲜";
    else if (!tfFresh()) reason = "map->base_link TF 不新鲜";
    else if (!ac_.isServerConnected()) reason = "move_base action 未就绪";
    if (why) *why = reason;
    return reason.empty();
  }
  void publishAuto(const geometry_msgs::Twist& twist) {
    geometry_msgs::TwistStamped out; out.header.stamp = ros::Time::now(); out.header.frame_id = "base_link";
    out.twist = twist; auto_pub_.publish(out);
  }
  void stop() { publishAuto(geometry_msgs::Twist()); }
  const char* stateName() const {
    switch (state_) {
      case State::WAIT_READY: return "WAIT_READY"; case State::NAVIGATING: return "NAVIGATING";
      case State::SEARCHING: return "SEARCHING"; case State::VISUAL: return "VISUAL";
      case State::PAUSED: return "PAUSED"; case State::FINISHED: return "FINISHED"; case State::FAULT: return "FAULT";
    }
    return "FAULT";
  }
  void publishState() { std_msgs::String msg; msg.data = stateName(); state_pub_.publish(msg); }
  void setState(State next, const std::string& reason) {
    if (state_ == next) return;
    state_ = next; publishState(); ROS_INFO("比赛状态 -> %s：%s", stateName(), reason.c_str());
    if (next == State::VISUAL) {
      visual_entered_ = ros::Time::now();
      // 进入视觉前清空上一次的状态/速度新鲜度，避免用旧数据判定 LOST。
      vision_status_recv_time_ = ros::Time();
      vision_status_stamp_ = ros::Time();
    }
  }
  void sendGoal() {
    move_base_msgs::MoveBaseGoal goal; goal.target_pose = waypoint_; goal.target_pose.header.stamp = ros::Time::now();
    ac_.sendGoal(goal); goal_sent_time_ = ros::Time::now(); auto_started_ = true;
    setState(State::NAVIGATING, "唯一预备点 goal 已发送");
  }
  void beginSearch() {
    search_started_ = ros::Time::now(); ++search_attempts_; search_accum_angle_ = 0.0;
    // 重置目标稳定计时起点。
    // 否则导航途中一直可见的蓝纸会让 target_first_time_ 停在很早的时刻，
    // 进入 SEARCHING 的第一次 tick 就误判"已稳定锁定"，搜索旋转一秒都没执行。
    if (targetFresh()) target_first_time_ = ros::Time::now();
    if (!tfFresh(&last_search_yaw_)) { setState(State::FAULT, "搜索开始时 TF 无效"); return; }
    setState(State::SEARCHING, "导航成功，开始有界旋转扫描");
  }
  void pauseForManual() {
    if (state_ == State::PAUSED || state_ == State::FINISHED || state_ == State::FAULT) return;
    state_before_pause_ = state_; if (state_ == State::NAVIGATING) ac_.cancelGoal(); stop();
    setState(State::PAUSED, "人工接管；释放后仍需显式 resume");
  }

  void tick() {
    if (manual_latched_ || (fresh(manual_time_, readiness_timeout_) && manual_active_)) { pauseForManual(); return; }
    if (state_ != State::WAIT_READY && state_ != State::PAUSED && state_ != State::FINISHED && state_ != State::FAULT &&
        !controlFreshAndReady()) {
      if (state_ == State::NAVIGATING) ac_.cancelGoal(); stop(); setState(State::FAULT, "control 心跳丢失或 ready=false"); return;
    }
    if (state_ != State::WAIT_READY && state_ != State::PAUSED && state_ != State::FINISHED && state_ != State::FAULT &&
        (!fresh(safety_ready_time_, readiness_timeout_) || !safety_ready_)) {
      if (state_ == State::NAVIGATING) ac_.cancelGoal(); stop(); setState(State::FAULT, "safety ready=false 或心跳丢失"); return;
    }
    if (state_ != State::WAIT_READY && state_ != State::PAUSED && state_ != State::FINISHED && state_ != State::FAULT &&
        (!fresh(localization_ready_time_, readiness_timeout_) || !localization_ready_)) {
      if (state_ == State::NAVIGATING) ac_.cancelGoal(); stop(); setState(State::FAULT, "localization ready=false 或心跳丢失"); return;
    }
    switch (state_) {
      case State::WAIT_READY: {
        stop(); std::string why;
        if (auto_start_ && !auto_started_ && ready(&why)) sendGoal();
        else if (!why.empty()) ROS_INFO_THROTTLE(3.0, "等待自动比赛就绪：%s", why.c_str());
        break;
      }
      case State::NAVIGATING: {
        if ((ros::Time::now() - goal_sent_time_).toSec() > goal_timeout_) {
          ac_.cancelGoal(); stop(); setState(State::FAULT, "导航 goal 超时"); break;
        }
        const auto action_state = ac_.getState();
        if (action_state == actionlib::SimpleClientGoalState::SUCCEEDED) { stop(); beginSearch(); break; }
        if (action_state == actionlib::SimpleClientGoalState::ABORTED || action_state == actionlib::SimpleClientGoalState::REJECTED ||
            action_state == actionlib::SimpleClientGoalState::LOST) {
          stop(); setState(State::FAULT, "导航 goal 失败：" + action_state.toString()); break;
        }
        if (allow_visual_during_navigation_ && targetFresh()) { ac_.cancelGoal(); stop(); setState(State::VISUAL, "显式允许途中视觉接管"); break; }
        if (fresh(nav_vel_time_, nav_cmd_timeout_)) publishAuto(nav_vel_); else stop();
        break;
      }
      case State::SEARCHING: {
        if (!fresh(scan_time_, readiness_timeout_)) { stop(); setState(State::FAULT, "搜索时 scan 失效"); break; }
        double yaw;
        if (!tfFresh(&yaw)) { search_accum_angle_ = 0.0; stop(); setState(State::FAULT, "搜索时 TF 失效，角度累计已清零"); break; }
        search_accum_angle_ += std::abs(std::atan2(std::sin(yaw - last_search_yaw_), std::cos(yaw - last_search_yaw_)));
        last_search_yaw_ = yaw;
        if (targetFresh()) {
          if ((ros::Time::now() - target_first_time_).toSec() >= target_stable_time_) { stop(); setState(State::VISUAL, "连续新鲜蓝目标已锁定"); }
          else stop();
        } else if ((ros::Time::now() - search_started_).toSec() >= search_max_time_ || search_accum_angle_ >= search_max_angle_) {
          stop();
          if (search_attempts_ >= search_max_attempts_) setState(State::FAULT, "搜索达到时间/角度/次数上限"); else beginSearch();
        } else { geometry_msgs::Twist cmd; cmd.angular.z = search_speed_; publishAuto(cmd); }
        break;
      }
      case State::VISUAL: {
        const bool status_fresh = fresh(vision_status_recv_time_, vision_cmd_timeout_) && fresh(vision_status_stamp_, target_timeout_);
        // 刚进入 VISUAL 时视觉状态尚未送达属正常，给一个宽限期等待首帧，
        // 避免"状态还没到"被当成目标丢失，瞬间消耗重试次数导致直接 FAULT。
        const bool visual_grace = !reached_since_.isZero() ? false :
            (ros::Time::now() - visual_entered_).toSec() < visual_grace_timeout_;
        if (!status_fresh && visual_grace && targetFresh() && vision_status_ != "FAULT") {
          stop(); break;
        }
        if (!targetFresh() || !status_fresh || vision_status_ == "LOST") {
          stop(); if (search_attempts_ < search_max_attempts_) beginSearch(); else setState(State::FAULT, "视觉目标丢失且重搜次数已用尽"); break;
        }
        if (vision_status_ == "FAULT") { stop(); setState(State::FAULT, "视觉伺服报告 FAULT"); break; }
        if (vision_status_ == "REACHED" && !reached_since_.isZero() &&
            (ros::Time::now() - reached_since_).toSec() >= reached_stable_time_) {
          stop(); setState(State::FINISHED, "视觉 REACHED 持续稳定"); break;
        }
        if (fresh(vision_vel_recv_time_, vision_cmd_timeout_) && fresh(vision_vel_.header.stamp, target_timeout_)) publishAuto(vision_vel_.twist);
        else stop();
        break;
      }
      case State::PAUSED: case State::FINISHED: case State::FAULT: stop(); break;
    }
  }
  bool startSrv(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    std::string why;
    if (state_ != State::WAIT_READY) { res.success=false; res.message="仅 WAIT_READY 可 start"; }
    else if (!ready(&why)) { res.success=false; res.message=why; }
    else { sendGoal(); res.success=true; res.message="比赛已启动"; }
    return true;
  }
  bool pauseSrv(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    if (state_ == State::NAVIGATING) ac_.cancelGoal(); state_before_pause_ = state_; stop(); setState(State::PAUSED, "收到 pause");
    res.success=true; res.message="已暂停"; return true;
  }
  bool resumeSrv(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    if (state_ != State::PAUSED) { res.success=false; res.message="当前未暂停"; return true; }
    if (manual_active_ || !controlFreshAndReady()) { res.success=false; res.message="人工使能未释放或 control 未就绪"; return true; }
    manual_latched_ = false;
    if (state_before_pause_ == State::NAVIGATING) { std::string why; if (!ready(&why)) { res.success=false; res.message=why; return true; } sendGoal(); }
    else if (state_before_pause_ == State::SEARCHING || state_before_pause_ == State::VISUAL) beginSearch();
    else setState(State::WAIT_READY, "恢复到安全等待");
    res.success=true; res.message="已恢复"; return true;
  }
  bool resetSrv(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
    ac_.cancelAllGoals(); stop(); auto_started_=false; search_attempts_=0; reached_since_=ros::Time(); manual_latched_=false;
    setState(State::WAIT_READY, "状态复位"); res.success=true; res.message="已复位"; return true;
  }

  ros::NodeHandle nh_, pnh_;
  MoveBaseClient ac_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Subscriber waypoint_sub_, nav_vel_sub_, target_sub_, vision_vel_sub_, vision_status_sub_, scan_sub_, control_ready_sub_, manual_sub_, safety_ready_sub_, localization_ready_sub_;
  ros::Publisher auto_pub_, state_pub_;
  ros::ServiceServer start_srv_, pause_srv_, resume_srv_, reset_srv_;
  State state_, state_before_pause_{State::WAIT_READY};
  geometry_msgs::PoseStamped waypoint_;
  geometry_msgs::Twist nav_vel_;
  geometry_msgs::TwistStamped vision_vel_;
  ros::Time nav_vel_time_, target_time_, target_first_time_, vision_vel_recv_time_, vision_status_stamp_, vision_status_recv_time_;
  ros::Time scan_time_, control_ready_time_, manual_time_, safety_ready_time_, localization_ready_time_, goal_sent_time_, search_started_, reached_since_;
  std::string vision_status_;
  bool waypoint_valid_{false}, control_ready_{false}, manual_active_{false}, manual_latched_{false}, safety_ready_{false}, localization_ready_{false}, auto_started_{false};
  bool auto_start_, allow_visual_during_navigation_;
  double readiness_timeout_, tf_timeout_, nav_cmd_timeout_, vision_cmd_timeout_, target_timeout_, goal_timeout_;
  double search_speed_, search_max_time_, search_max_angle_, target_stable_time_, reached_stable_time_;
  double visual_grace_timeout_;
  ros::Time visual_entered_;
  int search_max_attempts_, search_attempts_{0};
  double search_accum_angle_{0.0}, last_search_yaw_{0.0};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "race_state_machine_node"); RaceStateMachine node; node.run(); return 0;
}
