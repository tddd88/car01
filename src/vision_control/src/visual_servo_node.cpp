#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vision_control/servo_geometry.h>
#include <vision_control/parameters.h>
#include <memory>
#include <sstream>

class VisualServo {
public:
    VisualServo() : private_nh_("~"), tf_listener_(tf_buffer_) {
        using vision_control::parameter;
        config_.center_x = parameter(private_nh_, nh_, "center_x", 0.5);
        config_.lateral_sign = parameter(private_nh_, nh_, "lateral_sign", 1.0);
        config_.kp_x = parameter(private_nh_, nh_, "kp_x", 0.35);
        config_.kp_y = parameter(private_nh_, nh_, "kp_y", 0.45);
        config_.kp_yaw = parameter(private_nh_, nh_, "kp_yaw", 0.8);
        config_.max_vx = parameter(private_nh_, nh_, "max_vx", 0.10);
        config_.max_vy = parameter(private_nh_, nh_, "max_vy", 0.08);
        config_.max_wz = parameter(private_nh_, nh_, "max_wz", 0.20);
        config_.max_reverse = parameter(private_nh_, nh_, "max_reverse", 0.03);
        config_.near_speed = parameter(private_nh_, nh_, "near_speed", 0.04);
        config_.near_distance = parameter(private_nh_, nh_, "near_distance", 0.15);
        config_.accel_x = parameter(private_nh_, nh_, "accel_x", 0.15);
        config_.accel_y = parameter(private_nh_, nh_, "accel_y", 0.20);
        config_.accel_yaw = parameter(private_nh_, nh_, "accel_yaw", 0.40);
        config_.standoff = parameter(private_nh_, nh_, "standoff_distance", 0.30);
        config_.distance_tolerance = parameter(private_nh_, nh_, "distance_tolerance", 0.015);
        config_.center_tolerance = parameter(private_nh_, nh_, "center_tolerance", 0.025);
        config_.yaw_tolerance = parameter(private_nh_, nh_, "yaw_tolerance", 0.04);
        config_.stable_time = parameter(private_nh_, nh_, "stable_time", 0.8);
        config_.stable_frames = parameter(private_nh_, nh_, "stable_frames", 5);
        config_.allow_area_fallback = parameter(private_nh_, nh_, "allow_area_fallback", false);
        config_.target_area = parameter(private_nh_, nh_, "target_area", 0.30);
        wall_config_.inlier_tolerance = parameter(private_nh_, nh_, "wall_inlier_tolerance", 0.025);
        wall_config_.max_rmse = parameter(private_nh_, nh_, "wall_max_rmse", 0.020);
        wall_config_.min_width = parameter(private_nh_, nh_, "wall_min_width", 0.25);
        wall_config_.min_inlier_ratio = parameter(private_nh_, nh_, "wall_min_inlier_ratio", 0.65);
        wall_config_.max_yaw = parameter(private_nh_, nh_, "wall_max_yaw", 0.70);
        wall_config_.min_points = parameter(private_nh_, nh_, "wall_min_points", 8);
        target_timeout_ = parameter(private_nh_, nh_, "target_timeout", 0.3);
        scan_timeout_ = parameter(private_nh_, nh_, "scan_timeout", 0.3);
        publish_rate_ = parameter(private_nh_, nh_, "publish_rate", 20.0);
        front_sector_ = parameter(private_nh_, nh_, "wall_front_sector", 0.50);
        max_wall_range_ = parameter(private_nh_, nh_, "wall_max_range", 3.0);
        base_frame_ = parameter(private_nh_, nh_, "base_frame", std::string("base_link"));
        if (!validParameters()) throw std::runtime_error("视觉伺服参数无效，拒绝启动");
        controller_.reset(new vision_control::ServoController(config_));
        target_sub_ = nh_.subscribe("/vision/target", 1, &VisualServo::targetCb, this);
        scan_sub_ = nh_.subscribe(parameter(private_nh_, nh_, "scan_topic", std::string("/scan")),
                                  1, &VisualServo::scanCb, this);
        stamped_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/cmd_vel_vision_stamped", 1);
        legacy_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_vision", 1);
        status_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("/visual_servo/status", 1);
        timer_ = nh_.createTimer(ros::Duration(1.0 / publish_rate_), &VisualServo::tick, this);
    }

private:
    bool validParameters() const {
        const double finite_values[] = {config_.center_x, config_.lateral_sign, config_.kp_x,
            config_.kp_y, config_.kp_yaw, config_.max_vx, config_.max_vy, config_.max_wz,
            config_.max_reverse, config_.near_speed, config_.near_distance, config_.accel_x,
            config_.accel_y, config_.accel_yaw, config_.standoff, config_.distance_tolerance,
            config_.center_tolerance, config_.yaw_tolerance, config_.stable_time, config_.target_area,
            target_timeout_, scan_timeout_, publish_rate_, front_sector_, max_wall_range_,
            wall_config_.inlier_tolerance, wall_config_.max_rmse, wall_config_.min_width,
            wall_config_.min_inlier_ratio, wall_config_.max_yaw};
        for (double value : finite_values) if (!std::isfinite(value)) return false;
        return config_.center_x > 0.0 && config_.center_x < 1.0 &&
            std::abs(config_.lateral_sign) == 1.0 && config_.kp_x > 0.0 && config_.kp_y > 0.0 &&
            config_.kp_yaw > 0.0 && config_.max_vx > 0.0 && config_.max_vy > 0.0 && config_.max_wz > 0.0 &&
            config_.max_reverse >= 0.0 && config_.near_speed > 0.0 && config_.near_distance > 0.0 &&
            config_.accel_x > 0.0 && config_.accel_y > 0.0 && config_.accel_yaw > 0.0 &&
            config_.standoff > 0.0 && config_.distance_tolerance > 0.0 && config_.center_tolerance > 0.0 &&
            config_.yaw_tolerance > 0.0 && config_.stable_time >= 0.3 && config_.stable_frames >= 3 &&
            config_.target_area > 0.0 && config_.target_area < 1.0 && target_timeout_ > 0.0 &&
            target_timeout_ <= 0.3 && scan_timeout_ > 0.0 && scan_timeout_ <= 0.5 &&
            publish_rate_ >= 10.0 && publish_rate_ <= 100.0 && front_sector_ > 0.0 && front_sector_ < 1.57 &&
            max_wall_range_ > config_.standoff && wall_config_.inlier_tolerance > 0.0 &&
            wall_config_.max_rmse > 0.0 && wall_config_.min_width > 0.0 &&
            wall_config_.min_inlier_ratio > 0.5 && wall_config_.min_inlier_ratio <= 1.0 &&
            wall_config_.max_yaw > 0.0 && wall_config_.max_yaw < 1.57 && wall_config_.min_points >= 4;
    }

    bool fresh(const ros::Time& stamp, const ros::Time& received, double timeout,
               const ros::Time& now) const {
        double age = (now - stamp).toSec(), reception_age = (now - received).toSec();
        return !stamp.isZero() && age >= -0.05 && age <= timeout &&
               reception_age >= -0.05 && reception_age <= timeout;
    }

    void targetCb(const geometry_msgs::Vector3Stamped::ConstPtr& msg) {
        const auto& v = msg->vector;
        const ros::Time now = ros::Time::now();
        if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z) ||
            v.x < 0.0 || v.x > 1.0 || v.y < 0.0 || v.y > 1.0 || v.z <= 0.0 || v.z > 1.0 ||
            msg->header.frame_id.empty() || !fresh(msg->header.stamp, now, target_timeout_, now)) {
            target_fault_ = true;
            ROS_WARN_THROTTLE(1.0, "拒绝无效/过期视觉目标"); return;
        }
        // 队列重放与重复图像不能延长目标寿命，也不能计入到达稳态。
        if (target_seen_ && msg->header.stamp <= target_.header.stamp) return;
        target_ = *msg; target_received_ = now; target_seen_ = true; target_fault_ = false; ++target_seq_;
    }

    void scanCb(const sensor_msgs::LaserScan::ConstPtr& msg) {
        const ros::Time now = ros::Time::now();
        scan_good_ = false;
        if (msg->header.frame_id.empty() || !fresh(msg->header.stamp, now, scan_timeout_, now) ||
            !std::isfinite(msg->angle_min) || !std::isfinite(msg->angle_increment) || msg->angle_increment == 0.0 ||
            !std::isfinite(msg->range_min) || !std::isfinite(msg->range_max) ||
            msg->range_min < 0.0 || msg->range_max <= msg->range_min) return;
        tf2::Transform transform;
        transform.setIdentity();
        try {
            if (msg->header.frame_id != base_frame_) {
                auto tf = tf_buffer_.lookupTransform(base_frame_, msg->header.frame_id,
                                                     msg->header.stamp, ros::Duration(0.02));
                tf2::fromMsg(tf.transform, transform);
            }
        } catch (const tf2::TransformException& error) {
            ROS_WARN_THROTTLE(1.0, "雷达到 %s 的 TF 不可用: %s", base_frame_.c_str(), error.what()); return;
        }
        // 雷达安装高度不影响二维墙距；倾斜扫描平面则不能冒充水平几何测量。
        const tf2::Vector3 normal = transform.getBasis() * tf2::Vector3(0.0, 0.0, 1.0);
        if (std::hypot(normal.x(), normal.y()) > 0.15) return;
        std::vector<vision_control::Point2> points;
        const size_t stride = std::max<size_t>(1, msg->ranges.size() / 720);
        for (size_t i = 0; i < msg->ranges.size(); i += stride) {
            const double range = msg->ranges[i];
            if (!std::isfinite(range) || range <= msg->range_min || range >= msg->range_max) continue;
            const double angle = msg->angle_min + i * msg->angle_increment;
            tf2::Vector3 point = transform * tf2::Vector3(range * std::cos(angle), range * std::sin(angle), 0.0);
            if (point.x() > 0.05 && std::hypot(point.x(), point.y()) <= max_wall_range_ &&
                std::abs(std::atan2(point.y(), point.x())) <= front_sector_)
                points.push_back({point.x(), point.y()});
        }
        // 按角度顺序等距取样，将拟合开销限制为最多128点。
        std::vector<vision_control::Point2> sampled;
        if (points.size() > 128) {
            for (size_t i = 0; i < 128; ++i) sampled.push_back(points[i * (points.size() - 1) / 127]);
        } else sampled = points;
        wall_ = vision_control::fitWall(sampled, wall_config_);
        scan_stamp_ = msg->header.stamp; scan_received_ = now; scan_good_ = true;
    }

    static diagnostic_msgs::KeyValue value(const std::string& key, double number) {
        diagnostic_msgs::KeyValue output; output.key = key;
        std::ostringstream stream; stream << number; output.value = stream.str(); return output;
    }

    void tick(const ros::TimerEvent&) {
        const ros::Time now = ros::Time::now();
        bool target_fresh = target_seen_ && fresh(target_.header.stamp, target_received_, target_timeout_, now);
        bool scan_fresh = scan_good_ && fresh(scan_stamp_, scan_received_, scan_timeout_, now);
        vision_control::ServoOutput output;
        if (target_fault_) output = controller_->stop("FAULT", now.toSec());
        else output = controller_->step(now.toSec(), target_seen_, target_fresh, target_.vector.x,
                                       target_.vector.z, target_seq_, scan_fresh, wall_);
        geometry_msgs::TwistStamped speed;
        speed.header = target_.header;  // 速度与状态保留目标源时间，race 不能重放过期输入。
        speed.header.frame_id = base_frame_;
        speed.twist.linear.x = output.vx; speed.twist.linear.y = output.vy; speed.twist.angular.z = output.wz;
        stamped_pub_.publish(speed);
        legacy_pub_.publish(speed.twist);
        diagnostic_msgs::DiagnosticArray status;
        status.header = target_.header;
        diagnostic_msgs::DiagnosticStatus item;
        item.name = "visual_servo"; item.hardware_id = "blue_target_wall_servo"; item.message = output.state;
        item.level = output.state == "FAULT" ? diagnostic_msgs::DiagnosticStatus::ERROR :
                     (output.state == "LOST" || output.state == "WAITING" ? diagnostic_msgs::DiagnosticStatus::WARN :
                      diagnostic_msgs::DiagnosticStatus::OK);
        item.values.push_back(value("target_fresh", target_fresh));
        item.values.push_back(value("scan_fresh", scan_fresh));
        item.values.push_back(value("wall_valid", wall_.valid && scan_fresh));
        item.values.push_back(value("wall_distance_m", wall_.distance));
        item.values.push_back(value("wall_yaw_rad", wall_.yaw));
        item.values.push_back(value("wall_rmse_m", wall_.rmse));
        item.values.push_back(value("wall_inliers", wall_.inliers));
        item.values.push_back(value("center_error", config_.center_x - target_.vector.x));
        item.values.push_back(value("standoff_distance_m", config_.standoff));
        status.status.push_back(item); status_pub_.publish(status);
    }

    ros::NodeHandle nh_, private_nh_;
    ros::Subscriber target_sub_, scan_sub_;
    ros::Publisher stamped_pub_, legacy_pub_, status_pub_;
    ros::Timer timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    vision_control::ServoConfig config_;
    vision_control::WallConfig wall_config_;
    std::unique_ptr<vision_control::ServoController> controller_;
    vision_control::Wall wall_;
    geometry_msgs::Vector3Stamped target_;
    ros::Time target_received_, scan_received_, scan_stamp_;
    bool target_seen_ = false, target_fault_ = false, scan_good_ = false;
    unsigned long target_seq_ = 0;
    double target_timeout_, scan_timeout_, publish_rate_, front_sector_, max_wall_range_;
    std::string base_frame_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "visual_servo_node");
    try { VisualServo servo; ros::spin(); }
    catch (const std::exception& error) { ROS_FATAL("%s", error.what()); return 1; }
    return 0;
}
