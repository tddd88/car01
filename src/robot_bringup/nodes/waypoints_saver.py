#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Joy
import tf
import yaml
import os

class WaypointSaver:
    def __init__(self):
        self.listener = tf.TransformListener()
        # 绑定 B 键，在大多手柄中索引为 2 (A=0, B=1 或 A=1, B=2)，按需调整
        self.button_idx = rospy.get_param('~button_idx', 2)
        save_dir = rospy.get_param('~save_dir', '/home/ubuntu/car01/src/robot_slam/map')
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
        self.save_path = os.path.join(save_dir, 'vision_point.yaml')
        
        self.pressed = False
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_cb)
        rospy.loginfo("Waypoint saver ready. Press button %d to save current pose.", self.button_idx)

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
            
            data = {
                'vision_start_point': {
                    'x': float(trans[0]),
                    'y': float(trans[1]),
                    'yaw': float(euler[2])
                }
            }
            with open(self.save_path, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)
                
            rospy.loginfo("=======================================")
            rospy.loginfo("Waypoint SAVED to %s", self.save_path)
            rospy.loginfo("X: %.3f, Y: %.3f, YAW: %.3f", trans[0], trans[1], euler[2])
            rospy.loginfo("=======================================")
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("TF Error! Cannot save waypoint: %s", e)

if __name__ == '__main__':
    rospy.init_node('waypoint_saver_node')
    ws = WaypointSaver()
    rospy.spin()
