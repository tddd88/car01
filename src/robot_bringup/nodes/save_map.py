#!/usr/bin/env python3
"""手动保存 Cartographer 地图
用法：建图运行时，另开终端执行：
  python3 save_map.py
或指定路径：
  python3 save_map.py /path/to/map.pbstream
"""
import sys
import rospy
from cartographer_ros_msgs.srv import WriteState, WriteStateRequest

if __name__ == '__main__':
    map_path = sys.argv[1] if len(sys.argv) > 1 else \
        '/home/ubuntu/car01/src/robot_slam/map/map.pbstream'

    rospy.init_node('save_map_manual', anonymous=True)

    try:
        rospy.wait_for_service('/write_state', timeout=5.0)
    except rospy.ROSException:
        print("ERROR: /write_state service not found. Is cartographer_node running?")
        sys.exit(1)

    write_state = rospy.ServiceProxy('/write_state', WriteState)
    req = WriteStateRequest()
    req.filename = map_path
    req.include_unfinished_submaps = True

    print(f"Saving map to: {map_path}")
    resp = write_state(req)
    print(f"Status: {resp.status.message}")
    if resp.status.code != 0:
        print("ERROR: Cartographer rejected the save request")
        sys.exit(2)
    print("Done.")
