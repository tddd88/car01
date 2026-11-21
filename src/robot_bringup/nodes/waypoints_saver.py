#!/usr/bin/env python3
"""保存单个视觉预备点，并用 Cartographer 轨迹节点作为优化后重投影锚点。

2026-10-25 ZCode: 增加 legacy 回退。bundle 事务(active.yaml)只在走
mapping_session.sh 正式流程时才会生成; 直接建图 Ctrl+C 退出(含 Copilot 的
fallback_promote)只产出 map.pbstream + last_pose.yaml, 不产 active.yaml,
导致严格模式下比赛入口永远等不到预备点。现在:
  - 启动加载: active.yaml 存在且校验通过 -> 用版本化预备点;
    否则回退 save_dir/vision_point.yaml(legacy); 再否则等待 B 键现场保存。
  - B 键保存: 有 /mapping_asset(建图会话) -> 存 bundle 草稿(原逻辑);
    没有 -> 直接存 legacy vision_point.yaml 并发布, 导航测试时也能标点。
"""
import os
import yaml
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import Joy
from cartographer_ros_msgs.msg import TrajectoryStates
from cartographer_ros_msgs.srv import GetTrajectoryStates, TrajectoryQuery
from asset_bundle import (atomic_yaml, pose2d_from_msg, relative,
                          nearest_trajectory_pose, sha256_file, validate_pose_dict)


class WaypointSaver:
    def __init__(self):
        self.listener = tf.TransformListener()
        self.button_idx = rospy.get_param("~button_idx", 1)
        # 锚点容差：允许 B 点时刻与最近 Cartographer 轨迹节点的最大时间差。
        # 树莓派算力低时轨迹节点可能间隔 >1s，1.0 会让 B 键频繁保存失败；
        # 放宽到 3.0 以适配实际算力，代价是锚点重投影精度略降。
        self.anchor_tolerance = rospy.get_param("~anchor_tolerance", 3.0)
        # B 键保存失败时允许退化为"直接存当前 map->base_link 位姿"，
        # 避免因轨迹节点稀疏导致完全无法标点。
        self.allow_plain_fallback = rospy.get_param("~allow_plain_fallback", True)
        self.save_dir = rospy.get_param("~save_dir", "")
        self.pressed = False
        self.pub = rospy.Publisher("/vision_start_point", PoseStamped, latch=True, queue_size=1)
        self.joy_sub = rospy.Subscriber("/joy", Joy, self.joy_cb, queue_size=2)
        self.states = rospy.ServiceProxy("/get_trajectory_states", GetTrajectoryStates)
        self.query = rospy.ServiceProxy("/trajectory_query", TrajectoryQuery)
        if rospy.get_param("~load_active_bundle", False):
            self.load_active_bundle()
        rospy.loginfo("预备点保存器就绪；B键只覆盖本次 bundle 的单点草稿")

    def legacy_point_path(self):
        return os.path.join(self.save_dir, "vision_point.yaml")

    def current_map_identity(self):
        """读取当前地图身份 (map_id, map_sha256)，用于把 B 点绑定到具体地图版本。

        建图会话中：map.pbstream/last_pose.yaml 仍是上一张图，不能读文件！
        此时必须以 /mapping_asset 参数为准，sha256 暂时未知（返回 None，
        由最终兜底转正时回填）。
        非建图会话：优先用 last_pose.yaml（与地图配套写入），
        缺失时退化为直接哈希 map.pbstream。
        """
        # 建图会话优先：参数由 auto_save_map 在建图启动时设置
        session_id = rospy.get_param("/mapping_asset/map_id", "")
        session_staging = rospy.get_param("/mapping_asset/staging_dir", "")
        if session_id and session_staging:
            return session_id, None

        if not self.save_dir:
            return None, None
        map_path = os.path.join(self.save_dir, "map.pbstream")
        pose_path = os.path.join(self.save_dir, "last_pose.yaml")
        map_id = None
        try:
            with open(pose_path) as stream:
                doc = yaml.safe_load(stream) or {}
            map_id = doc.get("map_id")
            sha = doc.get("map_sha256")
            if sha:
                return map_id, sha
        except Exception:
            pass
        try:
            if os.path.isfile(map_path):
                return map_id, sha256_file(map_path)
        except Exception:
            pass
        return map_id, None

    def load_legacy_point(self, reason):
        path = self.legacy_point_path()
        if os.path.isfile(path):
            try:
                with open(path) as stream:
                    doc = yaml.safe_load(stream)
                pose = validate_pose_dict(doc["vision_start_point"])
                # 版本绑定校验：B 点必须属于当前这张地图，否则坐标会离谱地偏。
                cur_id, cur_sha = self.current_map_identity()
                point_id = doc.get("map_id")
                point_sha = doc.get("map_sha256")
                if not point_sha:
                    # 只有 map_id 能证明来源（建图期间标点、尚未回填哈希）。
                    if point_id and cur_id and point_id == cur_id:
                        self.publish(pose)
                        rospy.logwarn("%s；已回退发布 legacy 预备点（map_id 匹配，哈希待回填）：%s",
                                      reason, path)
                        return
                    rospy.logwarn(
                        "%s；legacy 预备点(%s)未绑定地图版本(map_id=%s 当前=%s)，"
                        "坐标可能属于旧图，已拒绝发布。请在当前地图上按 B 键重新标点。",
                        reason, path, point_id, cur_id)
                    return
                if cur_sha and point_sha != cur_sha:
                    rospy.logwarn(
                        "%s；legacy 预备点属于另一张地图(点=%s 当前=%s)，"
                        "已拒绝发布。请在当前地图上按 B 键重新标点。",
                        reason, point_sha[:12], cur_sha[:12])
                    return
                self.publish(pose)
                rospy.logwarn("%s；已回退发布 legacy 预备点（版本匹配）：%s", reason, path)
                return
            except Exception as exc:
                rospy.logerr("%s；且 legacy 预备点读取失败：%s", reason, exc)
                return
        rospy.logwarn("%s；且无 legacy vision_point.yaml，等待 B 键现场保存预备点", reason)

    def load_active_bundle(self):
        active_file = rospy.get_param("~active_file")
        if not os.path.isfile(active_file):
            self.load_legacy_point("没有 active bundle：%s" % active_file)
            return
        try:
            with open(active_file) as stream:
                active = yaml.safe_load(stream)
            with open(active["manifest"]) as stream:
                manifest = yaml.safe_load(stream)
            if not manifest.get("complete") or not manifest.get("point_valid"):
                raise ValueError("bundle 不完整或没有有效预备点")
            if active.get("map_id") != manifest.get("map_id"):
                raise ValueError("active/manifest map_id 不一致")
            bundle = os.path.dirname(active["manifest"])
            map_path = os.path.join(bundle, manifest["map_file"])
            if sha256_file(map_path) != manifest.get("map_sha256"):
                raise ValueError("地图 SHA256 不一致")
            with open(os.path.join(bundle, manifest["point_file"])) as stream:
                point = yaml.safe_load(stream)
            if point.get("map_id") != manifest["map_id"] or point.get("map_sha256") != manifest["map_sha256"]:
                raise ValueError("预备点版本与地图不一致")
            pose = validate_pose_dict(point["vision_start_point"])
            self.publish(pose)
            rospy.loginfo("已发布版本化预备点：map_id=%s", manifest["map_id"])
        except Exception as exc:
            self.load_legacy_point("预备点 bundle 校验失败：%s" % exc)

    def joy_cb(self, msg):
        if len(msg.buttons) <= self.button_idx:
            return
        down = msg.buttons[self.button_idx] == 1
        if down and not self.pressed:
            self.save_point()
        self.pressed = down

    def active_trajectory(self):
        response = self.states()
        if response.status.code != 0:
            raise RuntimeError("get_trajectory_states: %s" % response.status.message)
        pairs = zip(response.trajectory_states.trajectory_id,
                    response.trajectory_states.trajectory_state)
        active = [tid for tid, state in pairs if state == TrajectoryStates.ACTIVE]
        if len(active) != 1:
            raise RuntimeError("需要且只能有一条 ACTIVE 轨迹，实际 %s" % active)
        return active[0]

    def current_pose(self):
        self.listener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(1.0))
        trans, rot = self.listener.lookupTransform("/map", "/base_link", rospy.Time(0))
        return (float(trans[0]), float(trans[1]),
                float(tf.transformations.euler_from_quaternion(rot)[2]))

    def save_point(self):
        staging_dir = rospy.get_param("/mapping_asset/staging_dir", "")
        map_id = rospy.get_param("/mapping_asset/map_id", "")
        if not staging_dir or not map_id:
            # 非建图会话(导航测试/比赛前补标点): 直接存 legacy 点
            self.save_legacy_point()
            return
        try:
            stamp = self.listener.getLatestCommonTime("/map", "/base_link")
            base = self.current_pose()
            trajectory_id = self.active_trajectory()
            response = self.query(trajectory_id)
            if response.status.code != 0:
                raise RuntimeError("trajectory_query: %s" % response.status.message)
            anchor, error = nearest_trajectory_pose(response.trajectory, stamp.to_sec(), self.anchor_tolerance)
            anchor_pose = pose2d_from_msg(anchor.pose)
            rel = relative(anchor_pose, base)
            draft = {
                "schema": 1, "map_id": map_id, "valid": True,
                "trajectory_id": trajectory_id,
                "anchor_stamp": anchor.header.stamp.to_sec(),
                "capture_stamp": stamp.to_sec(), "anchor_error": error,
                "base_relative_to_anchor": {"x": rel[0], "y": rel[1], "yaw": rel[2]},
            }
            atomic_yaml(os.path.join(staging_dir, "vision_point.draft.yaml"), draft)
            # 2026-10-25 ZCode: 同步写 legacy 点。bundle 草稿只有在走 mapping_session.sh
            # 正式提交时才会变成可加载资产; 直接 Ctrl+C 退出的兜底只转正地图+位姿。
            # 双写保证"建图按 B -> 一键启动自动用"在两种退出方式下都成立。
            if self.save_dir:
                # B 点必须绑定当前地图版本，否则下次换了图会被误当成有效点使用。
                cur_id, cur_sha = self.current_map_identity()
                legacy_doc = {
                    "schema": 1, "map_id": cur_id or "legacy", "valid": True,
                    "map_sha256": cur_sha,
                    "vision_start_point": {"x": base[0], "y": base[1], "yaw": base[2]},
                }
                atomic_yaml(self.legacy_point_path(), legacy_doc)
            else:
                rospy.logwarn_throttle(10.0, "save_dir 未配置，本次 B 点只存 bundle 草稿，不写 legacy")
            self.publish(base)
            rospy.loginfo("预备点草稿已保存：map_id=%s x=%.3f y=%.3f yaw=%.3f", map_id, *base)
        except Exception as exc:
            # 锚点重投影失败（轨迹节点稀疏等）时退化为直接存当前位姿：
            # 树莓派算力低会让轨迹节点间隔超过容差，不能让 B 键彻底不可用。
            if self.allow_plain_fallback:
                rospy.logwarn("预备点锚点重投影失败(%s)，退化为直接保存当前位姿", exc)
                self.save_legacy_point()
            else:
                rospy.logerr("预备点未保存：%s", exc)

    def save_legacy_point(self):
        try:
            pose = self.current_pose()
            cur_id, cur_sha = self.current_map_identity()
            doc = {
                "schema": 1, "map_id": cur_id or "legacy", "valid": True,
                "map_sha256": cur_sha,
                "vision_start_point": {"x": pose[0], "y": pose[1], "yaw": pose[2]},
            }
            atomic_yaml(self.legacy_point_path(), doc)
            self.publish(pose)
            rospy.loginfo("预备点已保存：map_id=%s x=%.3f y=%.3f yaw=%.3f",
                          cur_id or "legacy", *pose)
        except Exception as exc:
            rospy.logerr("预备点未保存：%s", exc)

    def publish(self, pose):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.pose.position.x, msg.pose.position.y = pose[0], pose[1]
        msg.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, pose[2]))
        self.pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("waypoint_saver_node")
    WaypointSaver()
    rospy.spin()
