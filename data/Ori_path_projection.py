import rospy
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def project_onto_surface(points, mesh_path="surface.ply", sample_points=50000):
    # (1) 메쉬 로딩 및 샘플링
    mesh = o3d.io.read_triangle_mesh(mesh_path)
    mesh.compute_vertex_normals()
    pcd = mesh.sample_points_uniformly(number_of_points=sample_points)

    # (2) KDTree 구성
    pcd_tree = o3d.geometry.KDTreeFlann(pcd)

    # (3) 각 포인트를 메쉬에 가장 가까운 점으로 투영
    projected_points = []
    for pt in points:
        _, idx, _ = pcd_tree.search_knn_vector_3d(pt, 1)
        nearest = np.asarray(pcd.points)[idx[0]]
        projected_points.append(nearest)

    return np.array(projected_points)

def main():
    rospy.init_node('project_path_to_surface')

    # 파일 경로 설정
    ori_path_file = "/home/nrs/catkin_ws/src/nrs_path/data/Ori_path.txt"
    mesh_center_file = "/home/nrs/catkin_ws/src/nrs_path/data/Mesh_center.txt"
    mesh_file = "/home/nrs/catkin_ws/src/nrs_path/mesh/workpiece.stl"  # 메쉬 파일 경로
    aligned_path_file = "/home/nrs/catkin_ws/src/nrs_path/data/Ori_path_transformed.txt"
    projected_path_file = "/home/nrs/catkin_ws/src/nrs_path/data/Ori_path_projected.txt"

    # 원래 경로 불러오기
    old_path = np.loadtxt(ori_path_file, delimiter=" ")
    old_center = np.mean(old_path, axis=0)

    # 메쉬 중심점 불러오기
    try:
        new_center = np.loadtxt(mesh_center_file, delimiter=" ")
    except Exception as e:
        rospy.logerr(f"Failed to load new center from {mesh_center_file}: {e}")
        return

    # 중심 정렬
    aligned_path = old_path - old_center + new_center
    np.savetxt(aligned_path_file, aligned_path, fmt="%.6f")
    rospy.loginfo(f"Translated path saved to: {aligned_path_file}")

    # 메쉬 위로 전체 경로 투영
    projected_path = project_onto_surface(aligned_path, mesh_path=mesh_file)
    np.savetxt(projected_path_file, projected_path, fmt="%.6f")
    rospy.loginfo(f"Projected path saved to: {projected_path_file}")

    # 시각화
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(*old_path.T, label='Original Path', color='gray', linestyle='--', alpha=0.6)
    ax.plot(*aligned_path.T, label='Translated Path', color='blue')
    ax.plot(*projected_path.T, label='Projected on Surface', color='green')
    ax.scatter(*new_center, color='red', label='Mesh Center (From File)', s=50)

    ax.legend()
    ax.set_title("Path Projected onto Mesh Surface")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.view_init(elev=30, azim=45)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
