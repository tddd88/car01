#pragma once
#include <ros/ros.h>

namespace vision_control {
// 正式 launch 的节点私参优先；兼容旧独立预览放在 group 命名空间中的参数。
template <typename T>
T parameter(const ros::NodeHandle& private_nh, const ros::NodeHandle& public_nh,
            const std::string& name, const T& default_value) {
    T value;
    if (private_nh.getParam(name, value)) return value;
    if (public_nh.getParam(name, value)) return value;
    return default_value;
}
}
