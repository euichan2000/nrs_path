import numpy as np
import open3d as o3d
from scipy.signal import savgol_filter
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def reproject_to_surface(points, mesh_path="surface.ply", sample_points=300000):
    """
    주어진 points (Nx3 배열)를 메쉬에 재투영합니다.
    메쉬는 지정된 sample_points만큼 점을 샘플링하고, KDTree를 사용하여
    각 점의 가장 가까운 메쉬 포인트로 snap 시킵니다.
    """
    # 메쉬 로딩 및 포인트 클라우드 샘플링
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    mesh.compute_vertex_normals()
    pcd = mesh.sample_points_uniformly(number_of_points=sample_points)
    
    # KDTree 구성
    pcd_tree = o3d.geometry.KDTreeFlann(pcd)
    
    # 각 점 재투영 (nearest-neighbor 방식)
    reprojected_points = []
    for pt in points:
        _, idx, _ = pcd_tree.search_knn_vector_3d(pt, 1)
        nearest_pt = np.asarray(pcd.points)[idx[0]]
        reprojected_points.append(nearest_pt)
        
    return np.array(reprojected_points)

def smooth_projected_path(points, mesh_path, window_length=7, polyorder=3):
    """
    Savitzky-Golay 필터로 smoothing 후, 다시 메쉬 표면에 재투영합니다.
    """
    # smoothing: 각 축별 보간
    smooth_points = savgol_filter(points, window_length=window_length, polyorder=polyorder, axis=0)
    
    # smoothing 결과를 메쉬에 재투영
    smooth_reprojected = reproject_to_surface(smooth_points, mesh_path=mesh_path)
    return smooth_reprojected

def compute_fidelity(original, processed):
    """
    원본 경로와 처리된 경로 사이의 평균 Euclidean 오차(Mean Absolute Error)를 계산합니다.
    작은 값일수록 원본 의도가 잘 보존된 것으로 볼 수 있습니다.
    """
    # 각 점 사이 Euclidean distance 계산
    errors = np.linalg.norm(original - processed, axis=1)
    mae = np.mean(errors)
    return mae

def main():
    # 파일 경로 설정
    projected_path_file = "/home/nrs/catkin_ws/src/nrs_path/data/Ori_path_projected.txt"
    mesh_file = "/home/nrs/catkin_ws/src/nrs_path/mesh/workpiece.stl"
    smooth_output_file = "/home/nrs/catkin_ws/src/nrs_path/data/Projected_Path_Smooth.txt"
    
    # z 방향 offset (메쉬 아래로 내려가는 것을 방지하기 위해)
    z_offset = 0.005  # 5mm 정도, 필요에 따라 조정
    
    # 투영된 경로 불러오기 ([x, y, z] 형식)
    projected_path = np.loadtxt(projected_path_file, delimiter=" ")
    print(f"Loaded {projected_path.shape[0]} projected path points.")
    
    # z 방향으로 offset 적용 (모든 점에 대해)
    projected_path[:, 2] += z_offset
    
    # 초기 fidelity 평가 (투영된 경로 vs. 원본 투영 결과 – 이 경우 offset만 추가되므로 error가 z_offset 정도가 됨)
    initial_fidelity = compute_fidelity(projected_path, projected_path)
    print(f"Initial fidelity error: {initial_fidelity:.6f}")
    
    # Iterative smoothing + re-projection (반복 횟수 조절 가능)
    iterations = 3  # 예를 들어 3회 반복
    smooth_path = projected_path.copy()
    fidelity_history = []
    for i in range(iterations):
        print(f"Iteration {i+1} of smoothing...")
        smooth_path = smooth_projected_path(smooth_path, mesh_path=mesh_file,
                                            window_length=3, polyorder=2)
        # 다시 z offset 적용 후 fidelity 계산
        smooth_path[:, 2] += z_offset  
        fidelity_error = compute_fidelity(projected_path, smooth_path)
        fidelity_history.append(fidelity_error)
        print(f"Iteration {i+1} fidelity error: {fidelity_error:.6f}")
    
    # 최종 부드러운 경로 저장
    np.savetxt(smooth_output_file, smooth_path, fmt="%.6f")
    print(f"Smoothed projected path saved to {smooth_output_file}")
    
    # 시각화
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection="3d")
    # 원본 투영된 경로 (회색, 점선)
    ax.plot(projected_path[:,0], projected_path[:,1], projected_path[:,2],
            label="Original Projected Path", color="gray", linestyle="--", marker="o", alpha=0.5)
    # 최종 부드러운 경로 (파랑)
    ax.plot(smooth_path[:,0], smooth_path[:,1], smooth_path[:,2],
            label="Smoothed Path", color="blue", linestyle="-", marker="^")
    ax.set_title("Smoothed Path Projected onto Mesh Surface")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.view_init(elev=30, azim=45)
    # fidelity error 정보 표시 (최종 평가 값)
    final_fidelity = fidelity_history[-1]
    text_str = f"Final Fidelity Error (MAE): {final_fidelity:.4f}\nIterations: {iterations}\nWindow Length: 7, Polyorder: 2, z_offset: {z_offset}"
    ax.text2D(0.05, 0.95, text_str, transform=ax.transAxes, fontsize=12, color="red", 
              bbox=dict(facecolor="white", alpha=0.8))
    ax.legend()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
