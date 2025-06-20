import numpy as np
from sklearn.neighbors import NearestNeighbors
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def statistical_outlier_removal(points, k=10, std_ratio=1.0):
    """
    points: Nx3 numpy array (x, y, z)
    k: 주변 이웃 개수
    std_ratio: 표준편차 계수
    """
    nbrs = NearestNeighbors(n_neighbors=k + 1).fit(points)
    distances, _ = nbrs.kneighbors(points)

    # 자기 자신 제외한 평균 거리
    mean_dists = distances[:, 1:].mean(axis=1)

    # 평균과 표준편차 계산
    mean = np.mean(mean_dists)
    std = np.std(mean_dists)

    threshold_upper = mean + std_ratio * std
    mask = mean_dists < threshold_upper

    print(f"[SOR] Removed {np.sum(~mask)} outliers out of {len(points)} points")

    return points[mask]

# 파일 경로
input_file = "/home/nrs/catkin_ws/src/nrs_path/data/selected_waypoints.txt"
output_file = "/home/nrs/catkin_ws/src/nrs_path/data/sor_filtered_waypoints.txt"

# 데이터 읽기
points = np.loadtxt(input_file)  # Nx3 array

# Statistical Outlier Removal 적용
filtered_points = statistical_outlier_removal(points, k=5, std_ratio=1.0)

# 저장
np.savetxt(output_file, filtered_points, fmt="%.6f")
print(f"Filtered data saved to: {output_file}")

# 시각화
fig = plt.figure(figsize=(12, 6))

ax1 = fig.add_subplot(121, projection='3d')
ax1.scatter(points[:, 0], points[:, 1], points[:, 2], c='red', label='Raw')
ax1.set_title('Raw Points')
ax1.legend()

ax2 = fig.add_subplot(122, projection='3d')
ax2.scatter(filtered_points[:, 0], filtered_points[:, 1], filtered_points[:, 2], c='green', label='Filtered')
ax2.set_title('Filtered Points (SOR)')
ax2.legend()

plt.tight_layout()
plt.show()
