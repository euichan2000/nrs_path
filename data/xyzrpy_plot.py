import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # 3D plot을 위해 필요

# 데이터 파일 경로 (예: "data.txt")
filename = "/home/nrs/catkin_ws/src/nrs_path/data/selected_waypoints.txt"

# 공백을 기준으로 txt 파일 읽기 (각 행: x y z roll pitch yaw ...)
data = np.loadtxt(filename)

# 1~6열 추출
x = data[:, 0]
y = data[:, 1]
z = data[:, 2]
# roll  = data[:, 3]
# pitch = data[:, 4]
# yaw   = data[:, 5]

# Figure 생성 (좌측: 3D scatter, 우측: roll, pitch, yaw line plot)
fig = plt.figure(figsize=(12, 6))

# 3D 위치 시각화
ax1 = fig.add_subplot(121, projection='3d')
ax1.scatter(x, y, z, c='blue', marker='o')
ax1.set_title('3D Position')
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_zlabel('Z')
# 3D 위치 시각화
ax1 = fig.add_subplot(121, projection='3d')
ax1.scatter(x, y, z, c='blue', marker='o')
ax1.set_title('3D Position')
ax1.set_xlabel('X')
ax1.set_ylabel('Y')
ax1.set_zlabel('Z')

# === 축 스케일 고정 ===
max_range = np.array([
    x.max() - x.min(), 
    y.max() - y.min(), 
    z.max() - z.min()
]).max() / 2.0

mid_x = (x.max() + x.min()) * 0.5
mid_y = (y.max() + y.min()) * 0.5
mid_z = (z.max() + z.min()) * 0.5

ax1.set_xlim(mid_x - max_range, mid_x + max_range)
ax1.set_ylim(mid_y - max_range, mid_y + max_range)
ax1.set_zlim(mid_z - max_range, mid_z + max_range)

# roll, pitch, yaw 값 시각화
ax2 = fig.add_subplot(122)
#indices = np.arange(len(roll))  # 각 데이터의 인덱스 (또는 시간 축으로 활용 가능)
#ax2.plot(indices, roll, label='Roll')
# ax2.plot(indices, pitch, label='Pitch')
# ax2.plot(indices, yaw, label='Yaw')
ax2.set_title('Orientation Angles')
ax2.set_xlabel('Index')
ax2.set_ylabel('Angle (rad)')
ax2.legend()

plt.tight_layout()
plt.show()
