#!/usr/bin/env python3
import rospy
import os
import subprocess
import time

if __name__ == "__main__":
    rospy.init_node("wait")
    param_name = rospy.get_param("~param_name", "")
    launch_file = rospy.get_param("~launch_file", "")
    file_name = rospy.get_param("~file_name", "")
    reset_params = rospy.get_param("~reset_params", [])

    rospy.loginfo(f"Waiting for parameter: {param_name}")
    
    # 무제한 대기
    while not rospy.has_param(param_name) or not rospy.get_param(param_name):
        if rospy.is_shutdown():
            rospy.logwarn("Node is shutting down before the parameter was set.")
            exit(0)
        time.sleep(0.5)

    rospy.loginfo(f"Parameter {param_name} is now true. Launching next step.")
    subprocess.call(["roslaunch", launch_file, f"file_name:={file_name}"])
    
    # 지정된 파라미터 초기화
    rospy.loginfo("Resetting parameters...")
    for param in reset_params:
        rospy.set_param(param, False)
        rospy.loginfo(f"Reset parameter: {param}")
    