#!/usr/bin/env python3
"""隔离 ROS master 下的比赛状态机事件链测试；不连接硬件。"""
import threading
import time
import unittest
import rospy
import actionlib
import tf2_ros
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, TwistStamped, Vector3Stamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseResult
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class FakeWorld:
    def __init__(self):
        self.goals = 0
        self.succeed = threading.Event()
        self.server = actionlib.SimpleActionServer("move_base", MoveBaseAction,
                                                   execute_cb=self.execute, auto_start=False)
        self.server.start()
        self.pubs = {
            "point": rospy.Publisher("/vision_start_point", PoseStamped, queue_size=1, latch=True),
            "scan": rospy.Publisher("/scan", LaserScan, queue_size=2),
            "control": rospy.Publisher("/control/ready", Bool, queue_size=2),
            "manual": rospy.Publisher("/control/manual_active", Bool, queue_size=2),
            "safety": rospy.Publisher("/safety/ready", Bool, queue_size=2),
            "localization": rospy.Publisher("/localization/ready", Bool, queue_size=2),
            "target": rospy.Publisher("/vision/target", Vector3Stamped, queue_size=2),
            "vision_vel": rospy.Publisher("/cmd_vel_vision_stamped", TwistStamped, queue_size=2),
            "vision_status": rospy.Publisher("/visual_servo/status", DiagnosticArray, queue_size=2),
        }
        self.tf = tf2_ros.TransformBroadcaster()
        self.manual = False
        self.timer = rospy.Timer(rospy.Duration(0.03), self.heartbeat)

    def execute(self, _goal):
        self.goals += 1
        while not rospy.is_shutdown() and not self.succeed.wait(0.02):
            if self.server.is_preempt_requested():
                self.server.set_preempted(); return
        self.succeed.clear()
        self.server.set_succeeded(MoveBaseResult())

    def heartbeat(self, _event):
        now = rospy.Time.now()
        for key in ("control", "safety", "localization"):
            self.pubs[key].publish(Bool(True))
        self.pubs["manual"].publish(Bool(self.manual))
        scan = LaserScan(); scan.header.stamp = now; scan.header.frame_id = "laser"; self.pubs["scan"].publish(scan)
        transform = TransformStamped(); transform.header.stamp = now; transform.header.frame_id = "map"
        transform.child_frame_id = "base_link"; transform.transform.rotation.w = 1.0; self.tf.sendTransform(transform)

    def point(self):
        msg = PoseStamped(); msg.header.stamp = rospy.Time.now(); msg.header.frame_id = "map"; msg.pose.orientation.w = 1.0
        self.pubs["point"].publish(msg)

    def vision(self, status="TRACKING"):
        now = rospy.Time.now()
        target = Vector3Stamped(); target.header.stamp = now; target.vector.x=.5; target.vector.y=.5; target.vector.z=.2
        self.pubs["target"].publish(target)
        vel = TwistStamped(); vel.header.stamp = now; vel.twist.linear.x=.05; self.pubs["vision_vel"].publish(vel)
        diag = DiagnosticArray(); diag.header.stamp = now
        item = DiagnosticStatus(); item.name="visual_servo"; item.message=status; diag.status=[item]
        self.pubs["vision_status"].publish(diag)


class RaceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("race_fake_test")
        cls.world = FakeWorld()
        cls.state = None
        rospy.Subscriber("/race/state", String, lambda msg: setattr(cls, "state", msg.data))
        time.sleep(.5); cls.world.point()

    def wait_state(self, expected, timeout=4):
        limit = time.time() + timeout
        while time.time() < limit and not rospy.is_shutdown():
            if self.state == expected: return
            time.sleep(.02)
        self.fail("等待状态 %s 超时，当前 %s" % (expected, self.state))

    def test_01_complete_chain_and_ignore_early_blue(self):
        self.wait_state("NAVIGATING")
        self.world.vision("TRACKING"); time.sleep(.4)
        self.assertEqual(self.state, "NAVIGATING", "导航途中蓝色不应抢占")
        self.world.succeed.set(); self.wait_state("SEARCHING")
        end = time.time()+.7
        while time.time()<end: self.world.vision("TRACKING"); time.sleep(.05)
        self.wait_state("VISUAL")
        end = time.time()+1.1
        while time.time()<end: self.world.vision("REACHED"); time.sleep(.05)
        self.wait_state("FINISHED")

    def test_02_fast_manual_pulse_is_sticky(self):
        rospy.ServiceProxy("/race/reset", Trigger)(); self.world.point(); self.wait_state("NAVIGATING")
        self.world.manual=True; time.sleep(.04); self.world.manual=False
        self.wait_state("PAUSED")
        before=self.world.goals
        response=rospy.ServiceProxy("/race/resume", Trigger)()
        self.assertTrue(response.success); self.wait_state("NAVIGATING")
        self.assertGreater(self.world.goals, before)


if __name__ == "__main__":
    unittest.main()
