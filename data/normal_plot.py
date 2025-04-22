import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def rpy_to_zaxis(roll, pitch, yaw):
    """
    roll, pitch, yaw (in radians)로부터 회전된 z축 벡터를 계산합니다.
    (R = R_z(yaw)*R_y(pitch)*R_x(roll) 사용, 결과는 R*[0,0,1])
    """
    z_x = np.cos(yaw)*np.sin(pitch)*np.cos(roll) + np.sin(yaw)*np.sin(roll)
    z_y = np.sin(yaw)*np.sin(pitch)*np.cos(roll) - np.cos(yaw)*np.sin(roll)
    z_z = np.cos(pitch)*np.cos(roll)
    return np.array([z_x, z_y, z_z])

# 파일에서 데이터 읽기 (각 행: x, y, z, roll, pitch, yaw)
file_path = "/home/nrs/catkin_ws/src/nrs_path/data/geodesic_waypoints.txt"
data = np.loadtxt(file_path)

# x, y, z 좌표와 rpy 값 분리
points = data[:, :3]         # (x, y, z)
roll = data[:, 3]
pitch = data[:, 4]
yaw = data[:, 5]

# 각 행의 rpy로부터 z축 방향 벡터 계산
normals = np.array([rpy_to_zaxis(r, p, y) for r, p, y in zip(roll, pitch, yaw)])

# 10번째 줄마다 추출
sampled_points = points[::1]
sampled_normals = normals[::1]

# 3D 시각화
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# 점 그리기
ax.scatter(sampled_points[:, 0], sampled_points[:, 1], sampled_points[:, 2],
           c='r', label="Points")

# 방향 벡터 (z축 방향) 그리기
ax.quiver(sampled_points[:, 0], sampled_points[:, 1], sampled_points[:, 2],
          sampled_normals[:, 0], sampled_normals[:, 1], sampled_normals[:, 2],
          length=0.05, color='b', normalize=True)

ax.set_xlabel("X-axis")
ax.set_ylabel("Y-axis")
ax.set_zlabel("Z-axis")
ax.set_title("3D Points with Orientation Vectors (Every 10th Line)")
ax.legend()

plt.show()
