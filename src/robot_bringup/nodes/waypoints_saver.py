#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Joy
import tf
import yaml
import os
from geometry_msgs.msg import PoseStamped, Quaternion

class WaypointSaver:
    def __init__(self):
        self.listener = tf.TransformListener()
        # 绑定 B 键，X-Box 360 pad 中索引为 1
        self.button_idx = rospy.get_param('~button_idx', 1)
        save_dir = rospy.get_param('~save_dir', '/home/ubuntu/car01/src/robot_slam/map')
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        self.save_path = os.path.join(save_dir, 'vision_point.yaml')

        # latched 话题：保存预备点后发布一次，供 race_state_machine 读取并发起导航
        self.wp_pub = rospy.Publisher('/vision_start_point', PoseStamped, latch=True, queue_size=1)

        self.pressed = False
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_cb)

        # 启动时自动加载已保存的预备点并发布（比赛一键启动时无需再按手柄）
        self.load_and_publish_existing()
        rospy.loginfo("Waypoint saver ready. Press button %d to save current pose.", self.button_idx)

    def publish_waypoint(self, x, y, yaw):
        """将 (x,y,yaw) 发布为 latched /vision_start_point"""
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = rospy.Time.now()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = 0.0
        q = tf.transformations.quaternion_from_euler(0, 0, float(yaw))
        ps.pose.orientation = Quaternion(*q)
        self.wp_pub.publish(ps)

    def load_and_publish_existing(self):
        """读取已保存的 vision_point.yaml，若存在则自动发布预备点"""
        try:
            with open(self.save_path, 'r') as f:
                data = yaml.safe_load(f)
            wp = data.get('vision_start_point')
            if wp:
                self.publish_waypoint(wp['x'], wp['y'], wp['yaw'])
                rospy.loginfo("已自动加载并发布预备点: X=%.3f Y=%.3f YAW=%.3f",
                              wp['x'], wp['y'], wp['yaw'])
            else:
                rospy.logwarn("未在 %s 找到预备点，请用手柄 B 键保存。", self.save_path)
        except (IOError, TypeError) as e:
            rospy.logwarn("暂无可用的已存预备点 (%s)，比赛前请先建图并用 B 键标点。", e)

    def joy_cb(self, msg):
        if len(msg.buttons) > self.button_idx:
            if msg.buttons[self.button_idx] == 1 and not self.pressed:
                self.pressed = True
                self.save_point()
            elif msg.buttons[self.button_idx] == 0:
                self.pressed = False

    def save_point(self):
        try:
            # 获取那一瞬间的 TF 变换：从 map 坐标系 到 base_link 坐标系
            (trans, rot) = self.listener.lookupTransform('/map', '/base_link', rospy.Time(0))
            euler = tf.transformations.euler_from_quaternion(rot)
            yaw = float(euler[2])

            x = float(trans[0])
            y = float(trans[1])

            data = {
                'vision_start_point': {
                    'x': x,
                    'y': y,
                    'yaw': yaw
                }
            }
            with open(self.save_path, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)

            # 发布 latched 预备点给状态机
            self.publish_waypoint(x, y, yaw)

            rospy.loginfo("=======================================")
            rospy.loginfo("Waypoint SAVED to %s", self.save_path)
            rospy.loginfo("X: %.3f, Y: %.3f, YAW: %.3f", x, y, yaw)
            rospy.loginfo("=======================================")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("TF Error! Cannot save waypoint: %s", e)

if __name__ == '__main__':
    rospy.init_node('waypoint_saver_node')
    ws = WaypointSaver()
    rospy.spin()
