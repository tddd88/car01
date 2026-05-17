#!/usr/bin/env python3
import rospy
import time
from cartographer_ros_msgs.srv import WriteState, WriteStateRequest

def save_map():
    rospy.loginfo("---------------------------------------")
    rospy.loginfo("Catching shutdown signal. Auto-saving map...")
    map_path = rospy.get_param('~map_file', '/home/ubuntu/car01/src/robot_slam/map/map.pbstream')
    try:
        rospy.wait_for_service('/write_state', timeout=2.0)
        write_state = rospy.ServiceProxy('/write_state', WriteState)
        req = WriteStateRequest()
        req.filename = map_path
        resp = write_state(req)
        rospy.loginfo("Map successfully saved to: %s", map_path)
        rospy.loginfo("Status: %s", resp.status.message)
    except Exception as e:
        rospy.logerr("Auto-save failed! Error: %s", e)
    rospy.loginfo("---------------------------------------")

if __name__ == '__main__':
    rospy.init_node('auto_save_map', disable_signals=False)
    rospy.on_shutdown(save_map)
    rospy.loginfo("Auto save node initialized. Map will be saved on shutdown.")
    rospy.spin()
