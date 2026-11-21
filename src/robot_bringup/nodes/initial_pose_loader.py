#!/usr/bin/env python3
"""定位就绪/初始位姿节点。

两种模式(~inject_pose):
  false(默认, 2026-10-25 起): 就绪监视模式。不注入任何位姿,
      Cartographer 以默认轨迹从原点(=建图起始点)开始定位;
      本节点只等 Cartographer 出现 ACTIVE 轨迹后发布 /localization/ready=true,
      供比赛状态机的就绪门禁使用。不读 bundle/last_pose, 不碰任何轨迹。
  true: 保留的"注入建图退出位姿"模式(codex 版本)。需 bundles/active.yaml
      或 allow_legacy_assets=true 的 last_pose.yaml, 且要求启动时无 ACTIVE
      轨迹——历史实测经常因 Cartographer 已自带 ACTIVE 轨迹而拒绝启动,
      故默认不再使用, 仅留作以后恢复。
"""
import math
import os
import yaml
import rospy
from std_msgs.msg import Bool
from cartographer_ros_msgs.msg import TrajectoryStates
from cartographer_ros_msgs.srv import (StartTrajectory, StartTrajectoryRequest,
                                       GetTrajectoryStates, TrajectoryQuery)
from asset_bundle import compose, inverse, relative, sha256_file, validate_pose_dict, pose2d_from_msg


class InitialPoseLoader:
    def __init__(self):
        self.ready_pub = rospy.Publisher("/localization/ready", Bool, queue_size=1, latch=True)
        self.ready = False
        self.inject = rospy.get_param("~inject_pose", False)
        # 以下参数仅在注入模式使用; 默认模式不读
        self.allow_legacy = rospy.get_param("~allow_legacy_assets", False)
        self.pbstream = os.path.abspath(rospy.get_param("~pbstream", ""))
        self.active_file = os.path.abspath(rospy.get_param("~active_file", ""))
        self.config_directory = rospy.get_param("~config_directory", "")
        self.config_basename = rospy.get_param("~config_basename", "localization.lua")

    def load_assets(self):
        if os.path.isfile(self.active_file):
            with open(self.active_file) as stream:
                active = yaml.safe_load(stream)
            manifest_path = active.get("manifest")
            with open(manifest_path) as stream:
                manifest = yaml.safe_load(stream)
            bundle_dir = os.path.dirname(manifest_path)
            map_path = os.path.join(bundle_dir, manifest["map_file"])
            if not manifest.get("complete") or manifest.get("map_id") != active.get("map_id"):
                raise ValueError("active/manifest 不完整或 map_id 不一致")
            actual_sha = sha256_file(map_path)
            if actual_sha != manifest.get("map_sha256") or actual_sha != active.get("map_sha256"):
                raise ValueError("地图 SHA256 与 manifest/active 不一致")
            with open(os.path.join(bundle_dir, manifest["pose_file"])) as stream:
                pose_doc = yaml.safe_load(stream)
            if pose_doc.get("map_id") != manifest["map_id"] or pose_doc.get("map_sha256") != actual_sha:
                raise ValueError("last_pose 与地图版本不一致")
            return map_path, validate_pose_dict(pose_doc["last_pose"]), manifest["map_id"]
        if not self.allow_legacy:
            raise ValueError("没有 active bundle；严格模式拒绝旧地图")
        rospy.logwarn("显式使用 legacy 地图/pose；没有 map_id/SHA 绑定，仅供 nav_test 兼容")
        pose_path = os.path.join(os.path.dirname(self.pbstream), "last_pose.yaml")
        with open(pose_path) as stream:
            doc = yaml.safe_load(stream)
        return self.pbstream, validate_pose_dict(doc["last_pose"]), "legacy-unversioned"

    def start(self):
        if not self.inject:
            # 默认模式: Cartographer 自带默认轨迹(原点=建图起始点),
            # 本节点只负责确认轨迹活着并放行就绪门禁。
            rospy.wait_for_service("/get_trajectory_states", timeout=60.0)
            get_states = rospy.ServiceProxy("/get_trajectory_states", GetTrajectoryStates)
            rate = rospy.Rate(1.0)
            while not rospy.is_shutdown():
                try:
                    states = get_states()
                    if states.status.code == 0:
                        pairs = list(zip(states.trajectory_states.trajectory_id,
                                         states.trajectory_states.trajectory_state))
                        if any(s == TrajectoryStates.ACTIVE for _, s in pairs):
                            self.ready = True
                            rospy.loginfo("定位就绪(默认建图起始点模式): ACTIVE 轨迹已由 Cartographer 启动, 原点=建图起始点")
                            return
                except Exception as exc:
                    rospy.logwarn_throttle(5.0, "get_trajectory_states 暂不可用: %s", exc)
                rospy.loginfo_throttle(5.0, "等待 Cartographer 启动定位轨迹(原点=建图起始点)...")
                rate.sleep()
            return

        map_path, desired_global, map_id = self.load_assets()
        if os.path.realpath(map_path) != os.path.realpath(self.pbstream):
            rospy.loginfo("active bundle 地图覆盖传入 pbstream：%s", map_path)
        rospy.set_param("/localization/selected_pbstream", map_path)
        for service in ("/get_trajectory_states", "/trajectory_query", "/start_trajectory"):
            rospy.wait_for_service(service, timeout=30.0)
        states = rospy.ServiceProxy("/get_trajectory_states", GetTrajectoryStates)()
        if states.status.code != 0:
            raise RuntimeError("get_trajectory_states: %s" % states.status.message)
        pairs = list(zip(states.trajectory_states.trajectory_id,
                         states.trajectory_states.trajectory_state))
        active = [tid for tid, state in pairs if state == TrajectoryStates.ACTIVE]
        frozen = [tid for tid, state in pairs if state == TrajectoryStates.FROZEN]
        if active:
            raise RuntimeError("定位启动前已有 ACTIVE 轨迹：%s" % active)
        if len(frozen) != 1:
            raise RuntimeError("需要且只能有一条 FROZEN 地图轨迹，实际 %s" % frozen)
        ref_id = frozen[0]
        query = rospy.ServiceProxy("/trajectory_query", TrajectoryQuery)(ref_id)
        if query.status.code != 0 or not query.trajectory:
            raise RuntimeError("frozen trajectory query: %s" % query.status.message)

        # 本机 Cartographer PoseGraph2D::ComputeLocalToGlobalTransform 使用：
        # G_new = G_reference(time=0) * relative_pose，因此不能直接塞 map 绝对 pose。
        reference_global = pose2d_from_msg(query.trajectory[0].pose)
        relative_pose = relative(reference_global, desired_global)
        request = StartTrajectoryRequest()
        request.configuration_directory = self.config_directory
        request.configuration_basename = self.config_basename
        request.use_initial_pose = True
        request.relative_to_trajectory_id = ref_id
        request.initial_pose.position.x = relative_pose[0]
        request.initial_pose.position.y = relative_pose[1]
        request.initial_pose.orientation.z = math.sin(relative_pose[2] / 2.0)
        request.initial_pose.orientation.w = math.cos(relative_pose[2] / 2.0)
        response = rospy.ServiceProxy("/start_trajectory", StartTrajectory)(request)
        if response.status.code != 0:
            raise RuntimeError("start_trajectory: %s" % response.status.message)
        self.ready = True
        rospy.loginfo("定位轨迹 %d 已启动：map_id=%s desired=(%.3f,%.3f,%.3f) reference=(%.3f,%.3f,%.3f)",
                      response.trajectory_id, map_id, *(desired_global + reference_global))

    def spin(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.ready_pub.publish(Bool(self.ready))
            rate.sleep()


if __name__ == "__main__":
    rospy.init_node("initial_pose_loader")
    loader = InitialPoseLoader()
    try:
        loader.start()
    except Exception as exc:
        rospy.logerr("初始位姿加载失败：%s", exc)
    loader.spin()
