#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cmd_vel 续发/保活节点 (cmd_vel_repeat_node)

背景:
  树莓派算力不足时 move_base 的控制周期会被拉长(实测最差 ~1s/周期),
  而 STM32 有 300ms 指令超时看门狗 —— 超时即急停。
  于是导航变成 "走一下 -> 停 -> 再走一下", 且每次都从静止重新起步,
  起步瞬间轮端扭矩最大, 这也是打滑的重要来源。

作用:
  订阅上游速度(默认 /cmd_vel_nav), 以固定频率(默认 20Hz)向下游(默认 /cmd_vel)续发:
    * 上游速度非 0: 持续转发最近一次速度(给 move_base 输出做 20Hz 保活),
      move_base 偶尔慢一拍时底盘看门狗不会超时急停;
    * 上游速度为 0: 只发一次 0, 之后不刷(与 joy_repeat_node 下发规则一致);
    * 上游断流超过 max_hold(默认 0.5s): 视为规划器已停止输出, 发一次 0 兜底停车。

参数:
  ~input_topic   上游速度话题, 默认 /cmd_vel_nav
  ~output_topic  下游自动速度话题, 默认 /cmd_vel_auto (TwistStamped)
  ~rate          续发频率 Hz, 默认 20
  ~max_hold      非零速度最长续发时长 s, 默认 0.5(超过则自动停车)
"""

import rospy
from geometry_msgs.msg import Twist, TwistStamped


def is_zero(t):
    return t.linear.x == 0.0 and t.linear.y == 0.0 and t.angular.z == 0.0


class CmdVelRepeat(object):
    def __init__(self):
        input_topic = rospy.get_param("~input_topic", "/cmd_vel_nav")
        output_topic = rospy.get_param("~output_topic", "/cmd_vel_auto")
        rate = rospy.get_param("~rate", 20.0)
        self.max_hold = rospy.get_param("~max_hold", 0.5)

        self.last_cmd = None          # 最近一次上游速度
        self.last_cmd_time = None
        self.zero_sent = False        # 是否已经发过停车 0 速
        self.had_vel = False          # 是否处于"非零速度续发"状态

        self.pub = rospy.Publisher(output_topic, TwistStamped, queue_size=10)
        rospy.Subscriber(input_topic, Twist, self.cb, queue_size=5)
        rospy.Timer(rospy.Duration(1.0 / rate), self.tick)
        rospy.loginfo("cmd_vel_repeat: %s -> %s @%.1fHz, max_hold=%.2fs",
                      input_topic, output_topic, rate, self.max_hold)

    def cb(self, msg):
        self.last_cmd = msg
        self.last_cmd_time = rospy.Time.now()

    def publish(self, twist):
        out = TwistStamped()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = "base_link"
        out.twist = twist
        self.pub.publish(out)

    def tick(self, _):
        if self.last_cmd is None:
            return
        vel = self.last_cmd
        if not is_zero(vel):
            # 上游断流超时: 发一次 0 兜底停车
            if (rospy.Time.now() - self.last_cmd_time).to_sec() > self.max_hold:
                if not self.zero_sent:
                    self.publish(Twist())
                    self.zero_sent = True
                    self.had_vel = False
                    rospy.logwarn_throttle(2.0, "upstream cmd_vel stalled > %.2fs, stopping", self.max_hold)
                return
            # 非零速度: 20Hz 持续续发
            self.publish(vel)
            self.zero_sent = False
            self.had_vel = True
        else:
            # 上游为 0: 只发一次, 不持续刷 0
            if self.had_vel and not self.zero_sent:
                self.publish(Twist())
                self.zero_sent = True
            self.had_vel = False


if __name__ == "__main__":
    rospy.init_node("cmd_vel_repeat_node")
    CmdVelRepeat()
    rospy.spin()
