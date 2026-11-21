#!/usr/bin/env python3
"""地图资产 bundle 的纯函数与原子文件工具。"""
import hashlib
import math
import os
import tempfile
import yaml


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def atomic_yaml(path, data):
    directory = os.path.dirname(path)
    os.makedirs(directory, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix=".tmp-", dir=directory, text=True)
    try:
        with os.fdopen(fd, "w") as stream:
            yaml.safe_dump(data, stream, allow_unicode=True, sort_keys=True)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(tmp, path)
        dir_fd = os.open(directory, os.O_RDONLY)
        try:
            os.fsync(dir_fd)
        finally:
            os.close(dir_fd)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)


def pose2d_from_msg(pose):
    q = pose.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                     1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    return float(pose.position.x), float(pose.position.y), yaw


def compose(a, b):
    ca, sa = math.cos(a[2]), math.sin(a[2])
    return (a[0] + ca * b[0] - sa * b[1],
            a[1] + sa * b[0] + ca * b[1],
            normalize(a[2] + b[2]))


def inverse(a):
    ca, sa = math.cos(a[2]), math.sin(a[2])
    return (-ca * a[0] - sa * a[1], sa * a[0] - ca * a[1], normalize(-a[2]))


def relative(reference, target):
    return compose(inverse(reference), target)


def normalize(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def nearest_trajectory_pose(trajectory, stamp_sec, tolerance):
    if not trajectory:
        raise ValueError("轨迹没有可用节点")
    candidate = min(trajectory, key=lambda p: abs(p.header.stamp.to_sec() - stamp_sec))
    error = abs(candidate.header.stamp.to_sec() - stamp_sec)
    if error > tolerance:
        raise ValueError("最近轨迹节点相差 %.3fs，超过 %.3fs" % (error, tolerance))
    return candidate, error


def validate_pose_dict(data):
    if not isinstance(data, dict):
        raise ValueError("pose 不是对象")
    values = tuple(float(data[k]) for k in ("x", "y", "yaw"))
    if not all(math.isfinite(v) for v in values):
        raise ValueError("pose 含非有限值")
    return values
