#!/usr/bin/env python3
import rospy
import os
import subprocess
import signal
import time

if __name__ == "__main__":
    rospy.init_node("wait_and_terminate")
    
    # 파라미터 읽기
    param_name = rospy.get_param("~param_name", "")
    node_name = rospy.get_param("~node_name", "")  # 종료할 노드 이름
    launch_file = rospy.get_param("~launch_file", "")  # 종료할 런치 파일


    if not param_name or (not node_name and not launch_file):
        rospy.logerr("param_name, node_name or launch_file must be specified.")
        exit(1)

    rospy.loginfo(f"Waiting for parameter: {param_name}")
    
    # 무제한 대기
    while not rospy.has_param(param_name) or not rospy.get_param(param_name):
        if rospy.is_shutdown():
            rospy.logwarn("Node is shutting down before the parameter was set.")
            exit(0)
        time.sleep(0.5)

    rospy.loginfo(f"Parameter {param_name} is now true. Terminating target.")
    
    # 노드 종료
    if node_name:
        rospy.loginfo(f"Terminating node: {node_name}")
        try:
            # 노드 이름을 기준으로 PID 검색 및 종료
            output = subprocess.check_output(["rosnode", "list"], text=True)
            nodes = output.splitlines()
            if node_name in nodes:
                subprocess.call(["rosnode", "kill", node_name])
                rospy.loginfo(f"Node {node_name} terminated.")
            else:
                rospy.logwarn(f"Node {node_name} not found.")
        except Exception as e:
            rospy.logerr(f"Error terminating node {node_name}: {e}")

    # 런치 파일 종료 (launch process를 직접 종료해야 함)
    if launch_file:
        rospy.loginfo(f"Terminating launch file: {launch_file}")
        try:
            # 런치 파일의 프로세스를 찾고 종료
            ps_output = subprocess.check_output(["ps", "-e", "-o", "pid,cmd"], text=True)
            for line in ps_output.splitlines():
                if launch_file in line:
                    pid = int(line.split()[0])
                    os.kill(pid, signal.SIGTERM)
                    rospy.loginfo(f"Launch file {launch_file} terminated (PID: {pid}).")
                    break
            else:
                rospy.logwarn(f"Launch file {launch_file} process not found.")
        except Exception as e:
            rospy.logerr(f"Error terminating launch file {launch_file}: {e}")
