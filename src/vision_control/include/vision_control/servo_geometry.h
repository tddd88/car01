#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace vision_control {

inline double clamp(double value, double low, double high) {
    return std::max(low, std::min(high, value));
}

struct Point2 { double x; double y; };
struct Wall {
    bool valid = false;
    double distance = 0.0;
    double yaw = 0.0;
    double rmse = 0.0;
    double width = 0.0;
    int inliers = 0;
};
struct WallConfig {
    double inlier_tolerance = 0.025;
    double max_rmse = 0.020;
    double min_width = 0.25;
    double min_inlier_ratio = 0.65;
    double max_yaw = 0.70;
    int min_points = 8;
};

// 小规模确定性 RANSAC + 正交最小二乘，避免把多面墙/杂物当作可靠终点墙。
// points 已在 base_link 中，墙面法向朝机器人前方；距离以 base_link 原点为准。
inline Wall fitWall(const std::vector<Point2>& points, const WallConfig& config) {
    Wall result;
    const size_t count = points.size();
    if (count < static_cast<size_t>(config.min_points)) return result;
    std::vector<size_t> best;
    double best_error = std::numeric_limits<double>::infinity();
    const size_t anchors = std::min<size_t>(count, 12);
    for (size_t a = 0; a < anchors; ++a) {
        for (size_t b = a + 1; b < anchors; ++b) {
            size_t i = a * (count - 1) / (anchors - 1);
            size_t j = b * (count - 1) / (anchors - 1);
            double dx = points[j].x - points[i].x;
            double dy = points[j].y - points[i].y;
            double span = std::hypot(dx, dy);
            if (span < config.min_width * 0.5) continue;
            double nx = -dy / span, ny = dx / span;
            double d = nx * points[i].x + ny * points[i].y;
            if (nx < 0.0) { nx = -nx; ny = -ny; d = -d; }
            if (d <= 0.0 || std::abs(std::atan2(ny, nx)) > config.max_yaw) continue;
            std::vector<size_t> candidate;
            double error = 0.0;
            for (size_t k = 0; k < count; ++k) {
                double residual = std::abs(nx * points[k].x + ny * points[k].y - d);
                if (residual <= config.inlier_tolerance) {
                    candidate.push_back(k);
                    error += residual * residual;
                }
            }
            if (candidate.size() > best.size() ||
                (candidate.size() == best.size() && error < best_error)) {
                best = candidate;
                best_error = error;
            }
        }
    }
    if (best.size() < static_cast<size_t>(config.min_points) ||
        best.size() < config.min_inlier_ratio * count) return result;
    double mx = 0.0, my = 0.0;
    for (size_t k : best) { mx += points[k].x; my += points[k].y; }
    mx /= best.size(); my /= best.size();
    double xx = 0.0, xy = 0.0, yy = 0.0;
    for (size_t k : best) {
        double dx = points[k].x - mx, dy = points[k].y - my;
        xx += dx * dx; xy += dx * dy; yy += dy * dy;
    }
    double tangent = 0.5 * std::atan2(2.0 * xy, xx - yy);
    double nx = -std::sin(tangent), ny = std::cos(tangent);
    if (nx < 0.0) { nx = -nx; ny = -ny; }
    double d = nx * mx + ny * my;
    double error = 0.0, low = std::numeric_limits<double>::infinity(), high = -low;
    for (size_t k : best) {
        double residual = nx * points[k].x + ny * points[k].y - d;
        error += residual * residual;
        double along = -ny * points[k].x + nx * points[k].y;
        low = std::min(low, along); high = std::max(high, along);
    }
    result.distance = d;
    result.yaw = std::atan2(ny, nx);
    result.rmse = std::sqrt(error / best.size());
    result.width = high - low;
    result.inliers = best.size();
    result.valid = std::isfinite(d) && d > 0.0 && result.width >= config.min_width &&
                   result.rmse <= config.max_rmse && std::abs(result.yaw) <= config.max_yaw;
    return result;
}

struct ServoConfig {
    double center_x = 0.5;
    double lateral_sign = 1.0;
    double kp_x = 0.35, kp_y = 0.45, kp_yaw = 0.8;
    double max_vx = 0.10, max_vy = 0.08, max_wz = 0.20;
    double max_reverse = 0.03, near_speed = 0.04, near_distance = 0.15;
    double accel_x = 0.15, accel_y = 0.20, accel_yaw = 0.40;
    double standoff = 0.30, distance_tolerance = 0.015;
    double center_tolerance = 0.025, yaw_tolerance = 0.04;
    double stable_time = 0.8;
    int stable_frames = 5;
    bool allow_area_fallback = false;
    double target_area = 0.30;
};
struct ServoOutput {
    double vx = 0.0, vy = 0.0, wz = 0.0;
    std::string state = "WAITING";
};

class ServoController {
public:
    explicit ServoController(const ServoConfig& config) : config_(config) {}

    // target_seq 仅在新源图像时间戳到来时增加，重复消息不能凑满完成帧数。
    ServoOutput step(double now, bool target_seen, bool target_fresh, double center_x,
                     double area, unsigned long target_seq, bool scan_fresh, const Wall& wall) {
        ServoOutput desired;
        if (!target_seen) return stop("WAITING", now);
        if (!target_fresh) return stop("LOST", now);
        if (!scan_fresh) return stop("FAULT", now);
        double center_error = config_.center_x - center_x;
        if (!wall.valid) {
            resetStable();
            if (!config_.allow_area_fallback) return stop("FAULT", now);
            // 面积只允许保守接近，无法确定墙距/朝向，因此永不宣告 REACHED。
            desired.state = "TRACKING";
            if (std::abs(center_error) > config_.center_tolerance)
                desired.vy = config_.lateral_sign * clamp(config_.kp_y * center_error,
                                                        -config_.max_vy, config_.max_vy);
            else if (area < config_.target_area)
                desired.vx = std::min(config_.near_speed, config_.kp_x * (config_.target_area - area));
            return limit(desired, now);
        }
        const double distance_error = wall.distance - config_.standoff;
        const bool centered = std::abs(center_error) <= config_.center_tolerance;
        const bool facing = std::abs(wall.yaw) <= config_.yaw_tolerance;
        const bool at_distance = std::abs(distance_error) <= config_.distance_tolerance;
        if (centered && facing && at_distance) {
            if (stable_since_ < 0.0) { stable_since_ = now; frames_ = 0; stable_seq_ = target_seq - 1; }
            if (target_seq != stable_seq_) { stable_seq_ = target_seq; ++frames_; }
            if (now - stable_since_ >= config_.stable_time && frames_ >= config_.stable_frames)
                desired.state = "REACHED";
            else desired.state = "ALIGNED";
            // 在容差区内直接停稳；零速度不代表完成。
            last_ = desired; last_time_ = now;
            return desired;
        }
        resetStable();
        if (!facing) {
            desired.state = "TRACKING";
            desired.wz = clamp(config_.kp_yaw * wall.yaw, -config_.max_wz, config_.max_wz);
        } else if (!centered) {
            desired.state = "TRACKING";
            double lateral_limit = std::abs(distance_error) < config_.near_distance ?
                                   std::min(config_.near_speed, config_.max_vy) : config_.max_vy;
            desired.vy = config_.lateral_sign * clamp(config_.kp_y * center_error,
                                                    -lateral_limit, lateral_limit);
        } else {
            desired.state = "ALIGNED";
            double speed_limit = std::abs(distance_error) < config_.near_distance ?
                                 config_.near_speed : config_.max_vx;
            double brake_limit = std::sqrt(2.0 * config_.accel_x *
                                          std::max(0.0, std::abs(distance_error) - config_.distance_tolerance));
            speed_limit = std::min(speed_limit, brake_limit);
            desired.vx = clamp(config_.kp_x * distance_error,
                               -std::min(config_.max_reverse, speed_limit), speed_limit);
        }
        return limit(desired, now);
    }

    ServoOutput stop(const std::string& reason, double now) {
        resetStable(); last_ = ServoOutput(); last_.state = reason; last_time_ = now; return last_;
    }

private:
    void resetStable() { stable_since_ = -1.0; frames_ = 0; }
    ServoOutput limit(ServoOutput desired, double now) {
        double dt = last_time_ < 0.0 ? 0.0 : clamp(now - last_time_, 0.0, 0.1);
        // 每阶段只有一个自由度运动，其他轴立即置零，避免用同一像素误差同时偏航和横移。
        if (desired.vx != 0.0) desired.vx = clamp(desired.vx, last_.vx - config_.accel_x * dt, last_.vx + config_.accel_x * dt);
        if (desired.vy != 0.0) desired.vy = clamp(desired.vy, last_.vy - config_.accel_y * dt, last_.vy + config_.accel_y * dt);
        if (desired.wz != 0.0) desired.wz = clamp(desired.wz, last_.wz - config_.accel_yaw * dt, last_.wz + config_.accel_yaw * dt);
        last_ = desired; last_time_ = now; return desired;
    }
    ServoConfig config_;
    ServoOutput last_;
    double last_time_ = -1.0, stable_since_ = -1.0;
    unsigned long stable_seq_ = 0;
    int frames_ = 0;
};

}  // namespace vision_control
