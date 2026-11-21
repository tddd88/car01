#!/usr/bin/env python3
"""隔离 ROS 实测：真实源 header、TF 墙距、完成握手和过期数据停车。"""
import math
import os
import threading
import time
import unittest
from urllib.parse import urlparse

import rospy
import rostest
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Point, TwistStamped, Vector3Stamped
from sensor_msgs.msg import Image, LaserScan


class VisionInterfaces(unittest.TestCase):
    def setUp(self):
        self.assertEqual(urlparse(os.environ.get('ROS_MASTER_URI', '')).port, 11402,
                         '本测试必须使用隔离 master 11402')
        self.lock = threading.Lock()
        self.latest = {}
        self.history = []
        self.settings = dict(target=False, scan=True, center=0.5, area=0.1,
                             distance=0.6, yaw=0.0, stale_target=False, stale_scan=False,
                             invalid_target=False, scan_frame='test_laser')
        self.subscribers = []
        for key, topic, kind in [
            ('target', '/vision/target', Vector3Stamped),
            ('preview', '/test/preview_target', Vector3Stamped),
            ('legacy', '/blue_target', Point),
            ('debug', '/rgb/blue_target_debug_img', Image),
            ('speed', '/cmd_vel_vision_stamped', TwistStamped),
            ('status', '/visual_servo/status', DiagnosticArray),
        ]:
            self.subscribers.append(rospy.Subscriber(topic, kind, self.receive, key, queue_size=1))
        self.image_pub = rospy.Publisher('/test/image', Image, queue_size=1)
        self.target_pub = rospy.Publisher('/vision/target', Vector3Stamped, queue_size=1)
        self.scan_pub = rospy.Publisher('/test/scan', LaserScan, queue_size=1)
        self.running = True
        self.worker = threading.Thread(target=self.publish_scene, daemon=True)
        self.worker.start()

    def tearDown(self):
        self.running = False
        self.worker.join(timeout=2.0)
        for subscriber in self.subscribers:
            subscriber.unregister()

    def receive(self, message, key):
        with self.lock:
            self.latest[key] = message
            if key == 'status':
                self.history.append((time.monotonic(), message.status[0].message))

    def configure(self, **values):
        with self.lock:
            self.settings.update(values)
        time.sleep(0.1)

    def read(self, key):
        with self.lock:
            return self.latest.get(key)

    def state(self):
        message = self.read('status')
        return message.status[0].message if message and message.status else None

    def wait_for(self, condition, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if condition():
                return
            time.sleep(0.02)
        self.fail('等待条件超时，最新状态=%s' % self.state())

    def assert_stopped(self):
        speed = self.read('speed')
        self.assertIsNotNone(speed)
        self.assertEqual((speed.twist.linear.x, speed.twist.linear.y, speed.twist.angular.z),
                         (0.0, 0.0, 0.0))

    def make_scan(self, settings):
        msg = LaserScan()
        msg.header.stamp = rospy.Time.now() - rospy.Duration(0.6 if settings['stale_scan'] else 0.0)
        msg.header.frame_id = settings['scan_frame']
        msg.angle_min = -math.pi
        msg.angle_increment = 2 * math.pi / 720
        msg.angle_max = msg.angle_min + 719 * msg.angle_increment
        msg.range_min = 0.02
        msg.range_max = 5.0
        nx, ny = math.cos(settings['yaw']), math.sin(settings['yaw'])
        numerator = settings['distance'] - nx * 0.05 - ny * (-0.02)
        for i in range(720):
            angle = msg.angle_min + i * msg.angle_increment + 0.30
            denominator = nx * math.cos(angle) + ny * math.sin(angle)
            value = numerator / denominator if denominator > 0.02 else float('inf')
            msg.ranges.append(value if msg.range_min < value < msg.range_max else float('inf'))
        return msg

    def publish_scene(self):
        seq = 100
        while self.running and not rospy.is_shutdown():
            with self.lock:
                settings = dict(self.settings)
            if settings['scan']:
                self.scan_pub.publish(self.make_scan(settings))
            if settings['target']:
                msg = Vector3Stamped()
                msg.header.frame_id = 'test_camera'
                msg.header.stamp = rospy.Time.now() - rospy.Duration(0.6 if settings['stale_target'] else 0.0)
                msg.header.seq = seq
                msg.vector.x = float('nan') if settings['invalid_target'] else settings['center']
                msg.vector.y = 0.5
                msg.vector.z = settings['area']
                self.target_pub.publish(msg)
                seq += 1
            time.sleep(0.05)

    def test_end_to_end_boundaries(self):
        self.wait_for(lambda: self.state() == 'WAITING')
        self.assert_stopped()  # 首条视觉数据尚未来时也持续发明确零速。
        self.wait_for(lambda: self.image_pub.get_num_connections() == 2)
        image = Image()
        image.header.stamp = rospy.Time.now()
        image.header.frame_id = 'test_camera'
        image.header.seq = 42
        image.width, image.height, image.step = 160, 120, 480
        image.encoding = 'bgr8'
        pixels = bytearray(image.height * image.step)
        for row in range(50, 70):
            offset = row * image.step + 60 * 3
            pixels[offset:offset + 20 * 3] = b'\xff\x00\x00' * 20
        image.data = bytes(pixels)
        self.image_pub.publish(image)
        self.wait_for(lambda: self.read('debug') is not None and self.read('preview') is not None and self.read('target') is not None)
        target = self.read('target')
        self.assertEqual((target.header.frame_id, target.header.stamp), (image.header.frame_id, image.header.stamp))  # seq在rospy->roscpp->rospy往返不保真(实测C++端恒为0)，只断言关联字段
        self.assertAlmostEqual(target.vector.x, 69.5 / 160, places=6)
        self.assertAlmostEqual(target.vector.y, 59.5 / 120, places=6)
        self.assertAlmostEqual(target.vector.z, 361 / (160 * 120), places=6)
        self.assertEqual((self.read('debug').header.frame_id, self.read('debug').header.stamp), (image.header.frame_id, image.header.stamp))
        self.assertEqual((self.read('preview').header.frame_id, self.read('preview').header.stamp), (image.header.frame_id, image.header.stamp))  # 旧 group public 参数仍生效。
        self.assertAlmostEqual(self.read('legacy').x, target.vector.x, places=6)
        self.assertGreater(target.vector.z, 0.0)  # 私参面积100覆盖public10000与默认800。
        self.wait_for(lambda: self.state() == 'LOST')
        self.assert_stopped()

        # 偏置/转向雷达经真实 TF 后仍恢复正确墙距和角度，姿态先于横移/前进。
        self.configure(target=True, yaw=0.2, center=0.4)
        self.wait_for(lambda: self.read('speed').twist.angular.z > 0.01)
        speed = self.read('speed')
        self.assertEqual(speed.twist.linear.x, 0.0)
        self.assertEqual(speed.twist.linear.y, 0.0)
        status = self.read('status')
        values = {item.key: float(item.value) for item in status.status[0].values}
        self.assertAlmostEqual(values['wall_distance_m'], 0.6, delta=0.008)
        self.assertAlmostEqual(values['wall_yaw_rad'], 0.2, delta=0.015)
        self.assertEqual(status.status[0].name, 'visual_servo')
        self.assertLess(abs((speed.header.stamp - status.header.stamp).to_sec()), 0.11)

        self.configure(yaw=0.0, center=0.4)
        self.wait_for(lambda: self.read('speed').twist.linear.y > 0.01)
        self.assertEqual(self.read('speed').twist.angular.z, 0.0)
        self.assertEqual(self.read('speed').twist.linear.x, 0.0)
        self.configure(center=0.6)
        self.wait_for(lambda: self.read('speed').twist.linear.y < -0.01)
        self.configure(center=0.5)
        self.wait_for(lambda: self.read('speed').twist.linear.x > 0.01)
        self.assertEqual(self.read('speed').twist.linear.y, 0.0)

        # 大面积、零速或首次稳定帧都不能单独完成；真实墙距持续稳态才握手。
        self.configure(distance=0.30, center=0.2, area=0.9)
        time.sleep(1.0)
        self.assertNotEqual(self.state(), 'REACHED')
        self.configure(center=0.5, area=0.05)
        self.wait_for(lambda: self.state() == 'ALIGNED')
        self.assert_stopped()
        time.sleep(0.2)
        self.assertNotEqual(self.state(), 'REACHED')
        self.wait_for(lambda: self.state() == 'REACHED', timeout=2.0)
        self.assert_stopped()
        self.configure(target=False)
        lost_start = time.monotonic()
        self.wait_for(lambda: self.state() == 'LOST', timeout=0.45)
        self.assertLess(time.monotonic() - lost_start, 0.40)
        self.assert_stopped()
        self.configure(target=True)
        self.wait_for(lambda: self.state() == 'ALIGNED')
        self.assertNotEqual(self.state(), 'REACHED')

        # 持续重发过期采集时间、无效值、缺 TF 和旧扫描都立即失效清零。
        self.configure(stale_target=True)
        self.wait_for(lambda: self.state() == 'FAULT')
        self.assert_stopped()
        self.configure(stale_target=False, invalid_target=True)
        self.wait_for(lambda: self.state() == 'FAULT')
        self.assert_stopped()
        self.configure(invalid_target=False, stale_scan=True)
        self.wait_for(lambda: self.state() == 'FAULT')
        self.assert_stopped()
        self.configure(stale_scan=False, scan_frame='missing_laser_tf')
        self.wait_for(lambda: self.state() == 'FAULT')
        self.assert_stopped()
        self.configure(scan_frame='test_laser', distance=0.8)
        self.wait_for(lambda: self.read('speed').twist.linear.x > 0.01)
        self.configure(scan=False)
        self.wait_for(lambda: self.state() == 'FAULT', timeout=0.45)
        self.assert_stopped()


if __name__ == '__main__':
    rospy.init_node('vision_interfaces_test')
    rostest.rosrun('vision_control', 'vision_interfaces', VisionInterfaces)
