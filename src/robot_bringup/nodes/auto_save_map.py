#!/usr/bin/env python3
"""建立版本化地图 bundle；周期快照不结束轨迹，显式 finalize 才提交 active 指针。"""
import datetime
import os
import shutil
import threading
import time
import yaml
import rospy
import tf
from std_srvs.srv import Trigger, TriggerResponse
from cartographer_ros_msgs.msg import TrajectoryStates
from cartographer_ros_msgs.srv import (WriteState, WriteStateRequest, FinishTrajectory,
                                       GetTrajectoryStates, TrajectoryQuery)
from asset_bundle import (atomic_yaml, compose, nearest_trajectory_pose, pose2d_from_msg,
                          relative, sha256_file, validate_pose_dict)


class MapBundleSaver:
    def __init__(self):
        self.root = os.path.abspath(rospy.get_param("~bundle_root"))
        self.save_interval = float(rospy.get_param("~save_interval", 5.0))
        self.anchor_tolerance = float(rospy.get_param("~anchor_tolerance", 1.0))
        self.optimization_wait = float(rospy.get_param("~optimization_wait", 2.0))
        self.map_id = rospy.get_param("~map_id", "") or datetime.datetime.utcnow().strftime("map-%Y%m%dT%H%M%SZ")
        self.staging = rospy.get_param("~staging_dir", "") or os.path.join(self.root, ".staging-" + self.map_id)
        self.final_state_file = rospy.get_param("~final_state_file", os.path.join(self.staging, ".final.pbstream.tmp"))
        self.final_query_service = rospy.get_param("~final_query_service", "/wincodex_finalize/trajectory_query")
        self.final_dir = os.path.join(self.root, self.map_id)
        os.makedirs(self.staging, exist_ok=True)
        unexpected = [name for name in os.listdir(self.staging)
                      if name not in (os.path.basename(self.final_state_file),)]
        if unexpected:
            raise RuntimeError("staging 目录不是空的新会话：%s" % unexpected)
        rospy.set_param("/mapping_asset/map_id", self.map_id)
        rospy.set_param("/mapping_asset/staging_dir", self.staging)
        self.listener = tf.TransformListener()
        self.write = rospy.ServiceProxy("/write_state", WriteState)
        self.finish = rospy.ServiceProxy("/finish_trajectory", FinishTrajectory)
        self.states = rospy.ServiceProxy("/get_trajectory_states", GetTrajectoryStates)
        self.query = rospy.ServiceProxy("/trajectory_query", TrajectoryQuery)
        self.lock = threading.Lock()
        self.finalized = False
        self.timer = rospy.Timer(rospy.Duration(self.save_interval), self.periodic)
        self.prepared = False
        self.prepare_srv = rospy.Service("/mapping/prepare_finalize", Trigger, self.prepare_finalize)
        self.commit_srv = rospy.Service("/mapping/commit_final", Trigger, self.commit_final)
        rospy.loginfo("新建 mapping bundle staging：%s", self.staging)

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

    def write_state(self, path, unfinished):
        request = WriteStateRequest(filename=path, include_unfinished_submaps=unfinished)
        response = self.write(request)
        if response.status.code != 0:
            raise RuntimeError("write_state: %s" % response.status.message)
        if not os.path.isfile(path) or os.path.getsize(path) == 0:
            raise RuntimeError("write_state 返回成功但文件为空")

    def periodic(self, _event):
        if self.finalized or not self.lock.acquire(False):
            return
        try:
            tmp = os.path.join(self.staging, ".snapshot.pbstream.tmp")
            self.write_state(tmp, True)
            target = os.path.join(self.staging, "snapshot.pbstream")
            os.replace(tmp, target)
            snap_yaml = os.path.join(self.staging, "snapshot.yaml")
            # 保留上一次的位姿：本次取位姿失败时不至于把好数据覆盖掉
            old_pose = None
            if os.path.isfile(snap_yaml):
                try:
                    with open(snap_yaml) as stream:
                        old_pose = (yaml.safe_load(stream) or {}).get("pose")
                except Exception:
                    old_pose = None
            meta = {
                "schema": 1, "map_id": self.map_id, "complete": True,
                "sha256": sha256_file(target), "stamp": rospy.Time.now().to_sec(),
                "include_unfinished_submaps": True,
            }
            # 与快照同一时刻记录 map->base_link 位姿，供兜底转正写 last_pose。
            # 必须在建图期间取：Ctrl+C 退出时 TF 往往已失效。
            try:
                self.listener.waitForTransform("/map", "/base_link",
                                               rospy.Time(0), rospy.Duration(0.5))
                trans, rot = self.listener.lookupTransform("/map", "/base_link",
                                                           rospy.Time(0))
                meta["pose"] = {
                    "x": float(trans[0]), "y": float(trans[1]),
                    "yaw": float(tf.transformations.euler_from_quaternion(rot)[2]),
                }
            except Exception as exc:
                rospy.logwarn_throttle(10.0, "快照位姿记录失败（沿用上次）：%s", exc)
                if old_pose:
                    meta["pose"] = old_pose
            atomic_yaml(snap_yaml, meta)
        except Exception as exc:
            rospy.logerr("周期地图快照失败：%s", exc)
        finally:
            self.lock.release()

    def capture_anchor(self, trajectory_id):
        self.listener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(1.0))
        trans, rot = self.listener.lookupTransform("/map", "/base_link", rospy.Time(0))
        stamp = self.listener.getLatestCommonTime("/map", "/base_link")
        base = (float(trans[0]), float(trans[1]), float(tf.transformations.euler_from_quaternion(rot)[2]))
        response = self.query(trajectory_id)
        if response.status.code != 0:
            raise RuntimeError("trajectory_query: %s" % response.status.message)
        anchor, error = nearest_trajectory_pose(response.trajectory, stamp.to_sec(), self.anchor_tolerance)
        return {"trajectory_id": trajectory_id, "anchor_stamp": anchor.header.stamp.to_sec(),
                "anchor_error": error, "base_relative_to_anchor": relative(pose2d_from_msg(anchor.pose), base)}

    def project(self, record, trajectory):
        anchor, error = nearest_trajectory_pose(trajectory, float(record["anchor_stamp"]), self.anchor_tolerance)
        rel = validate_pose_dict(record["base_relative_to_anchor"])
        pose = compose(pose2d_from_msg(anchor.pose), rel)
        return {"x": pose[0], "y": pose[1], "yaw": pose[2], "anchor_error": error}

    def prepare_finalize(self, _request):
        with self.lock:
            if self.finalized or self.prepared:
                return TriggerResponse(False, "本次 bundle 已 prepare/finalize")
            try:
                tid = self.active_trajectory()
                pose_anchor = self.capture_anchor(tid)
                point_draft_path = os.path.join(self.staging, "vision_point.draft.yaml")
                point_draft = None
                if os.path.isfile(point_draft_path):
                    with open(point_draft_path) as stream:
                        point_draft = yaml.safe_load(stream)
                    if point_draft.get("map_id") != self.map_id or point_draft.get("trajectory_id") != tid:
                        raise RuntimeError("预备点草稿与当前 map_id/trajectory 不一致")
                if not point_draft:
                    raise RuntimeError("本次 bundle 尚未成功保存 B 预备点，拒绝 prepare")

                finish_response = self.finish(tid)
                if finish_response.status.code != 0:
                    raise RuntimeError("finish_trajectory: %s" % finish_response.status.message)
                atomic_yaml(os.path.join(self.staging, "finalize_anchors.yaml"), {
                    "schema": 1, "map_id": self.map_id, "trajectory_id": tid,
                    "last_pose_anchor": pose_anchor, "point_anchor": point_draft,
                })
                self.prepared = True
                return TriggerResponse(True, "已冻结轨迹与锚点；现在须优雅退出 cartographer 运行最终优化")
            except Exception as exc:
                rospy.logerr("prepare finalize 失败：%s", exc)
                return TriggerResponse(False, str(exc))

    def commit_final(self, _request):
        with self.lock:
            if self.finalized:
                return TriggerResponse(False, "本次 bundle 已提交")
            try:
                if not self.prepared:
                    raise RuntimeError("尚未 prepare finalize")
                if not os.path.isfile(self.final_state_file) or os.path.getsize(self.final_state_file) == 0:
                    raise RuntimeError("Cartographer 最终优化输出不存在或为空")
                with open(os.path.join(self.staging, "finalize_anchors.yaml")) as stream:
                    anchors = yaml.safe_load(stream)
                tid = int(anchors["trajectory_id"])
                rospy.wait_for_service(self.final_query_service, timeout=15.0)
                query_response = rospy.ServiceProxy(self.final_query_service, TrajectoryQuery)(tid)
                if query_response.status.code != 0 or not query_response.trajectory:
                    raise RuntimeError("重新加载最终地图后的 trajectory_query 失败：%s" % query_response.status.message)
                map_path = os.path.join(self.staging, "map.pbstream")
                os.replace(self.final_state_file, map_path)
                map_sha = sha256_file(map_path)
                final_pose = self.project(anchors["last_pose_anchor"], query_response.trajectory)
                atomic_yaml(os.path.join(self.staging, "last_pose.yaml"), {
                    "schema": 1, "map_id": self.map_id, "map_sha256": map_sha,
                    "last_pose": final_pose,
                })
                final_point = self.project(anchors["point_anchor"], query_response.trajectory)
                atomic_yaml(os.path.join(self.staging, "vision_point.yaml"), {
                    "schema": 1, "map_id": self.map_id, "map_sha256": map_sha,
                    "valid": True, "vision_start_point": final_point,
                })
                point_valid = True

                manifest = {"schema": 1, "map_id": self.map_id, "complete": True,
                            "map_file": "map.pbstream", "map_sha256": map_sha,
                            "pose_file": "last_pose.yaml", "point_file": "vision_point.yaml" if point_valid else None,
                            "point_valid": point_valid, "finalized_at": rospy.Time.now().to_sec()}
                atomic_yaml(os.path.join(self.staging, "manifest.yaml"), manifest)
                os.rename(self.staging, self.final_dir)
                link_tmp = os.path.join(self.root, ".active_map.pbstream.tmp")
                if os.path.lexists(link_tmp):
                    os.unlink(link_tmp)
                os.symlink(os.path.relpath(os.path.join(self.final_dir, "map.pbstream"), self.root), link_tmp)
                os.replace(link_tmp, os.path.join(self.root, "active_map.pbstream"))
                # active.yaml 是事务提交点；它最后更新，读取方先校验 symlink/hash/manifest。
                atomic_yaml(os.path.join(self.root, "active.yaml"), {
                    "schema": 1, "map_id": self.map_id, "bundle_dir": self.final_dir,
                    "manifest": os.path.join(self.final_dir, "manifest.yaml"), "map_sha256": map_sha,
                })
                self.finalized = True
                rospy.set_param("/mapping_asset/staging_dir", "")
                return TriggerResponse(True, "最终优化 bundle 已原子提交：%s" % self.final_dir)
            except Exception as exc:
                rospy.logerr("commit final 失败；active 指针保持原版本：%s", exc)
                return TriggerResponse(False, str(exc))

    def _backfill_vision_point(self, map_id, map_sha):
        """把本次会话的 map_id/sha256 回填到 vision_point.yaml。

        建图期间按 B 写点无法预知最终地图哈希，这里在转正后补齐，
        使 B 点与最终地图版本绑定（load_legacy_point 的校验依赖它）。
        """
        if not map_id or not map_sha:
            return
        point_path = os.path.join(os.path.dirname(self.root), "vision_point.yaml")
        if not os.path.isfile(point_path):
            rospy.loginfo("本次建图未按 B 键标点，跳过预备点回填")
            return
        try:
            with open(point_path) as stream:
                doc = yaml.safe_load(stream) or {}
            point_id = doc.get("map_id")
            # 只回填属于本次会话的点；别的会话的点不动。
            if point_id and point_id != map_id:
                rospy.logwarn("vision_point.yaml 属于 %s（本次 %s），不回填", point_id, map_id)
                return
            if doc.get("map_sha256") == map_sha:
                return
            doc["map_id"] = map_id
            doc["map_sha256"] = map_sha
            atomic_yaml(point_path, doc)
            rospy.loginfo("已回填预备点地图版本：map_id=%s sha=%s", map_id, map_sha[:12])
        except Exception as exc:
            rospy.logwarn("预备点回填失败：%s", exc)

    def fallback_promote(self):
        """兜底：没走正式 finalize（如直接 Ctrl+C）时，用最近的周期快照转正。

        校验 staging 快照的 SHA256 与 snapshot.yaml 一致后：
          1. 备份旧 map.pbstream 为 map.pbstream.bak-<时间戳>
          2. 快照替换 map.pbstream（legacy 主地图，localization 可直接加载）
          3. 更新 last_pose.yaml（若不存在才写，正式流程写得更准）
        这样任何退出方式都保证"刚建的图"可用。
        """
        if self.finalized:
            return
        snap_pbstream = os.path.join(self.staging, "snapshot.pbstream")
        snap_yaml = os.path.join(self.staging, "snapshot.yaml")
        if not (os.path.isfile(snap_pbstream) and os.path.isfile(snap_yaml)):
            rospy.logwarn("无周期快照可转正，保留原地图")
            return
        try:
            with open(snap_yaml) as stream:
                meta = yaml.safe_load(stream)
            actual = sha256_file(snap_pbstream)
            if meta.get("sha256") != actual:
                raise RuntimeError("快照 SHA256 不一致：%s != %s" % (meta.get("sha256"), actual))
            legacy_map = os.path.join(os.path.dirname(self.root), "map.pbstream")
            if os.path.isfile(legacy_map):
                backup = legacy_map + ".bak-" + time.strftime("%Y%m%dT%H%M%S")
                shutil.copy2(legacy_map, backup)
                rospy.loginfo("旧地图已备份：%s", backup)
            shutil.copy2(snap_pbstream, legacy_map)
            rospy.loginfo("兜底转正完成：%s -> %s（map_id=%s）", snap_pbstream, legacy_map, meta.get("map_id"))
            # 回填预备点的地图哈希。
            # 建图期间按 B 写入的点只能知道 map_id（当时 map.pbstream 还是上一张图，
            # 无法预知最终哈希）。转正后这里把本会话的 map_id/sha 回填，
            # 否则加载时版本校验会一直拒绝这个点。
            self._backfill_vision_point(meta.get("map_id"), actual)
            # 位姿必须与地图配套更新，否则导航起始位置会错。
            # 优先用快照里记录的位姿（建图期间实时采集，可靠）；
            # 没有才退化为当前 TF（退出时多半取不到）。
            pose_path = os.path.join(os.path.dirname(self.root), "last_pose.yaml")
            snap_pose = meta.get("pose")
            if snap_pose:
                atomic_yaml(pose_path, {
                    "schema": 1, "map_id": meta.get("map_id"),
                    "map_sha256": actual, "last_pose": snap_pose,
                })
                rospy.loginfo("已写入 last_pose.yaml（快照位姿 X=%.3f Y=%.3f YAW=%.3f）",
                              snap_pose["x"], snap_pose["y"], snap_pose["yaw"])
                return
            try:
                self.listener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(1.0))
                trans, rot = self.listener.lookupTransform("/map", "/base_link", rospy.Time(0))
                yaw = float(tf.transformations.euler_from_quaternion(rot)[2])
                atomic_yaml(pose_path, {
                    "schema": 1, "map_id": meta.get("map_id"),
                    "map_sha256": actual,
                    "last_pose": {"x": float(trans[0]), "y": float(trans[1]), "yaw": yaw},
                })
                rospy.loginfo("已写入 last_pose.yaml（退出时 TF 兜底）")
            except Exception as exc:
                rospy.logwarn("last_pose 兜底写入失败（CW 时 TF 通常已失效）：%s", exc)
        except Exception as exc:
            rospy.logerr("快照兜底转正失败，保留原地图：%s", exc)


if __name__ == "__main__":
    rospy.init_node("map_bundle_saver")
    saver = MapBundleSaver()

    def _shutdown():
        # 正式 finalize 已提交则什么都不做；否则用快照兜底转正
        saver.fallback_promote()

    rospy.on_shutdown(_shutdown)
    rospy.spin()
