#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import filtfilt
import rospy
from std_srvs.srv import Empty, EmptyResponse

def interpolate_callback(req):
    rospy.loginfo("Interpolation service called...")

    # ===== 1. 파일 읽기 =====
    file_path = "/home/nrs/catkin_ws/src/nrs_path/data/continuous_teaching_waypoints.txt"
    xyz = np.loadtxt(file_path)
    N = xyz.shape[0]

    # ===== 2. Statistical Outlier Removal =====
    k = 5
    distances = np.zeros(N)
    for i in range(N):
        diffs = xyz - xyz[i, :]
        dists = np.linalg.norm(diffs, axis=1)
        dists[i] = np.nan
        sorted_dists = np.sort(dists[~np.isnan(dists)])
        distances[i] = np.mean(sorted_dists[:k])

    mean_dist = np.nanmean(distances)
    std_dist = np.nanstd(distances)
    z_scores = (distances - mean_dist) / std_dist
    inlier_idx = np.abs(z_scores) < 2.0
    xyz_inlier = xyz[inlier_idx]

    # ===== 3. 선형 보간 =====
    original_idx = np.where(inlier_idx)[0]
    full_idx = np.arange(N)
    xyz_interp = xyz.copy()
    for dim in range(3):
        xyz_interp[~inlier_idx, dim] = np.interp(
            full_idx[~inlier_idx], full_idx[inlier_idx], xyz[inlier_idx, dim]
        )

    # ===== 4. 이동 평균 필터 =====
    window_size = 10
    b = np.ones(window_size) / window_size
    x_smooth = filtfilt(b, [1.0], xyz_interp[:, 0])
    y_smooth = filtfilt(b, [1.0], xyz_interp[:, 1])
    z_smooth = filtfilt(b, [1.0], xyz_interp[:, 2])
    xyz_smooth = np.vstack((x_smooth, y_smooth, z_smooth)).T

    # ===== 5. 저장 =====
    output_path = "/home/nrs/catkin_ws/src/nrs_path/data/interpolated_continuous_waypoints.txt"
    np.savetxt(output_path, xyz_smooth, fmt="%.6f")
    rospy.loginfo("Smoothed data saved to: %s", output_path)

    return EmptyResponse()


if __name__ == "__main__":
    rospy.init_node("nrs_node_interpolate_continuous_waypoints")
    service = rospy.Service("/interpolate_continuous_waypoints", Empty, interpolate_callback)
    rospy.loginfo("Service [/interpolate_continuous_waypoints] ready.")
    rospy.spin()
