#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
import open3d as o3d
import numpy as np
import copy
import yaml
import os

# PLY 파일 로드 및 파싱
def load_ply(file_path):
    vertices = []
    normals = []
    plane_indices = []
    colors = []
    with open(file_path, "r") as file:
        lines = file.readlines()

    # 헤더 처리
    header_end_idx = 0
    for i, line in enumerate(lines):
        if line.strip() == "end_header":
            header_end_idx = i + 1
            break

    # 데이터 처리
    for line in lines[header_end_idx:]:
        values = line.strip().split()
        x, y, z = map(float, values[0:3])  # 좌표
        nx, ny, nz = map(float, values[3:6])  # 노말
        plane_index = int(values[6])
        if plane_index == 18446744073709551615:  # -1 처리
            plane_index = -1
        vertices.append([x, y, z])
        normals.append([nx, ny, nz])
        plane_indices.append(plane_index)
        r, g, b = map(int, values[7:10])  # 색상 (0-255 범위)
        colors.append([r / 255.0, g / 255.0, b / 255.0])  # 0-1로 정규화

    return (
        np.array(vertices),
        np.array(normals),
        np.array(plane_indices),
        np.array(colors),
    )


def create_point_cloud(vertices, colors, normals):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(vertices)
    pcd.colors = o3d.utility.Vector3dVector(colors)
    pcd.normals = o3d.utility.Vector3dVector(normals)
    return pcd


def pick_points(pcd):
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(pcd)
    vis.run()  # user picks points
    vis.destroy_window()
    return vis.get_picked_points()


def set_correspondence_point(cloud, vertices, plane_indices, target_plane_index):
    """
    선택한 평면의 중심점(centroid)과 전체 포인트 클라우드의 중심점을 계산하고 이를 가상 점으로 추가합니다.

    Parameters:
        cloud (o3d.geometry.PointCloud): 현재 포인트 클라우드.
        vertices (np.ndarray): 점들의 좌표 배열 (Nx3).
        plane_indices (np.ndarray): 각 점의 평면 인덱스 배열 (Nx1).
        target_plane_index (int): 선택한 점의 평면 인덱스.

    Returns:
        tuple: (업데이트된 포인트 클라우드, 평면 중심 메시, 전체 중심 메시, 선택한 점 인덱스, 선택한 점의 평면 중심 인덱스, 전체 point cloud중심 인덱스)
    """

    # 선택한 평면의 중심점과 해당 인덱스 계산
    def compute_plane_centroid(vertices, plane_indices, target_plane_index):
        plane_points = vertices[plane_indices == target_plane_index]
        if len(plane_points) == 0:
            raise ValueError(f"No points found for plane index {target_plane_index}")
        centroid = np.mean(plane_points, axis=0)
        plane_indices_mask = np.where(plane_indices == target_plane_index)[0]
        distances = np.linalg.norm(vertices[plane_indices_mask] - centroid, axis=1)
        closest_point_index = plane_indices_mask[np.argmin(distances)]
        return centroid, closest_point_index

    # 전체 포인트 클라우드의 중심점 계산
    def compute_overall_centroid(vertices):
        overall_centroid = np.mean(vertices, axis=0)
        overall_distances = np.linalg.norm(vertices - overall_centroid, axis=1)
        overall_centroid_index = np.argmin(overall_distances)
        return overall_centroid, overall_centroid_index

    def add_point(original_cloud, centroid):
        centroid_cloud = o3d.geometry.PointCloud()
        centroid_cloud.points = o3d.utility.Vector3dVector(np.array([centroid]))
        original_cloud = original_cloud + centroid_cloud

    # 선택한 평면의 중심점 계산
    plane_centroid, plane_centroid_index = compute_plane_centroid(
        vertices, plane_indices, target_plane_index
    )
    overall_centroid, overall_centroid_index = compute_overall_centroid(vertices)

    # 기존 포인트클라우드에 centroid 점 추가
    add_point(cloud, plane_centroid)
    add_point(cloud, overall_centroid)

    # 평면 중심 메시 생성
    plane_centroid_mesh = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
    plane_centroid_mesh.translate(plane_centroid)
    plane_centroid_mesh.paint_uniform_color([0, 1, 0])  # Green for plane centroid mesh

    # 전체 중심 메시 생성
    overall_centroid_mesh = o3d.geometry.TriangleMesh.create_sphere(radius=0.01)
    overall_centroid_mesh.translate(overall_centroid)
    overall_centroid_mesh.paint_uniform_color(
        [1, 0, 0]
    )  # Red for overall centroid mesh

    return (
        cloud,
        plane_centroid_mesh,
        overall_centroid_mesh,
        plane_centroid_index,
        overall_centroid_index,
    )


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


def rotate_point_cloud_z_axis(pcd, angle_degrees):
    """
    주어진 PointCloud 객체를 Z축 기준으로 회전합니다.

    Parameters:
        pcd (o3d.geometry.PointCloud): 회전할 PointCloud 객체
        angle_degrees (float): 회전 각도 (단위: 도)

    Returns:
        o3d.geometry.PointCloud: 회전된 PointCloud 객체
    """
    # 각도를 라디안으로 변환
    angle_radians = np.deg2rad(angle_degrees)

    # Z축 회전 변환 행렬 생성
    rotation_matrix = np.array(
        [
            [np.cos(angle_radians), -np.sin(angle_radians), 0],
            [np.sin(angle_radians), np.cos(angle_radians), 0],
            [0, 0, 1],
        ]
    )

    # 점들을 회전
    rotated_points = np.asarray(pcd.points) @ rotation_matrix.T

    # 회전된 점들로 새로운 PointCloud 생성
    rotated_pcd = o3d.geometry.PointCloud()
    rotated_pcd.points = o3d.utility.Vector3dVector(rotated_points)
    rotated_pcd.colors = pcd.colors  # 색상 유지

    return rotated_pcd


def align_centroid(source_cloud, target_cloud):
    """
    두 포인트 클라우드의 중심점을 기반으로 정렬합니다.

    Parameters:
        source_cloud (o3d.geometry.PointCloud): 이동시킬 Source PointCloud
        target_cloud (o3d.geometry.PointCloud): 기준이 되는 Target PointCloud

    Returns:
        o3d.geometry.PointCloud: 정렬된 Source PointCloud
        np.ndarray: 변환 행렬 (4x4)
    """
    # Source와 Target의 중심점(centroid) 계산
    src_centroid = np.mean(np.asarray(source_cloud.points), axis=0)
    tgt_centroid = np.mean(np.asarray(target_cloud.points), axis=0)

    # 중심점 차이 계산
    translation = tgt_centroid - src_centroid

    # 변환 행렬 생성 (4x4 행렬)
    transform = np.identity(4)
    transform[:3, 3] = translation  # translation 벡터 적용

    # Source PointCloud에 변환 적용
    aligned_source_cloud = copy.deepcopy(source_cloud)
    aligned_source_cloud.transform(transform)

    return aligned_source_cloud


def save_to_yaml(file_path, key, value):
    """
    YAML 파일에 key-value 데이터를 저장합니다.
    기존 파일이 존재하면 병합하여 업데이트합니다.

    Parameters:
        file_path (str): YAML 파일 경로.
        key (str): 저장할 키.
        value: 저장할 값 (4x4 행렬을 1D 리스트로 변환).
    """
    data = {}
    # 기존 YAML 파일 읽기
    if os.path.exists(file_path):
        with open(file_path, "r") as file:
            try:
                data = yaml.safe_load(file) or {}
            except yaml.YAMLError as exc:
                print(f"Error reading YAML file: {exc}")

    # value가 2D 리스트(4x4 행렬)인 경우 1D 리스트로 변환
    if isinstance(value, list) and isinstance(value[0], list):
        value = [item for row in value for item in row]

    # 새 데이터 추가
    data[key] = value

    # YAML 파일 쓰기
    with open(file_path, "w") as file:
        yaml.safe_dump(data, file)

def main():
    rospy.init_node("registration_node", anonymous=True)


    src = rospy.get_param("registration_node/source_file")
    tgt = rospy.get_param("registration_node/target_file")

    # PLY 파일 로드 및 데이터 파싱
    src_vertices, src_normals, src_plane_indices, src_colors = load_ply(src)
    tgt_vertices, tgt_normals, tgt_plane_indices, tgt_colors = load_ply(tgt)

    src_cloud = create_point_cloud(src_vertices, src_colors, src_normals)
    tgt_cloud = create_point_cloud(tgt_vertices, tgt_colors, tgt_normals)

    #src_cloud = rotate_point_cloud_z_axis(src_cloud, 80)

    o3d.visualization.draw_geometries(
        [src_cloud, tgt_cloud],
        window_name="PointCloud with Centroid",
    )

    if not rospy.get_param("/use_rough_registration"):
        src_cloud = align_centroid(src_cloud, tgt_cloud)
        init_transform = np.identity(4)
        # print(init_transform)
        print("Perform point-to-point ICP refinement")
        threshold = 0.1

        # ICP convergence criteria 설정
        criteria = o3d.pipelines.registration.ICPConvergenceCriteria(
            relative_fitness=1e-6,  # fitness 기준 수렴 오차
            relative_rmse=1e-6,  # RMSE 기준 수렴 오차
            max_iteration=100,  # 최대 반복 횟수
        )
        reg_p2p = o3d.pipelines.registration.registration_icp(
            src_cloud,
            tgt_cloud,
            threshold,
            init_transform,
            o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            criteria,
        )
        draw_registration_result(src_cloud, tgt_cloud, reg_p2p.transformation)
        rmse = reg_p2p.inlier_rmse

    else:
        while True:
            # 선택한 점의 index
            tgt_picked_id = pick_points(tgt_cloud)
            src_picked_id = pick_points(src_cloud)

            # 선택한 점이 위치한 평면의 index
            src_target_plane_index = src_plane_indices[src_picked_id][0]
            tgt_target_plane_index = tgt_plane_indices[tgt_picked_id][0]
            # correspondence 점 설정
            (
                src_cloud,
                src_plane_mesh,
                src_overall_mesh,
                src_plane_index,
                src_overall_index,
            ) = set_correspondence_point(
                src_cloud, src_vertices, src_plane_indices, src_target_plane_index
            )
            (
                tgt_cloud,
                tgt_plane_mesh,
                tgt_overall_mesh,
                tgt_plane_index,
                tgt_overall_index,
            ) = set_correspondence_point(
                tgt_cloud, tgt_vertices, tgt_plane_indices, tgt_target_plane_index
            )

            # 결과 시각화
            # o3d.visualization.draw_geometries(
            #     [
            #         src_cloud,
            #         src_plane_mesh,
            #         src_overall_mesh,
            #         tgt_cloud,
            #         tgt_plane_mesh,
            #         tgt_overall_mesh,
            #     ],
            #     window_name="PointCloud with Centroid",
            # )

            src_clicked_point_index = src_picked_id[0]
            tgt_clicked_point_index = tgt_picked_id[0]

            corr = np.zeros((3, 2), dtype=int)

            src_tmp = [src_clicked_point_index, src_plane_index, src_overall_index]
            tgt_tmp = [tgt_clicked_point_index, tgt_plane_index, tgt_overall_index]

            corr[:, 0] = np.array(src_tmp)
            corr[:, 1] = np.array(tgt_tmp)

            p2p = o3d.pipelines.registration.TransformationEstimationPointToPoint()
            init_transform = p2p.compute_transformation(
                src_cloud, tgt_cloud, o3d.utility.Vector2iVector(corr)
            )
            # print(init_transform)
            print("Perform point-to-point ICP refinement")
            threshold = 0.1

            # ICP convergence criteria 설정
            criteria = o3d.pipelines.registration.ICPConvergenceCriteria(
                relative_fitness=1e-6,  # fitness 기준 수렴 오차
                relative_rmse=1e-6,  # RMSE 기준 수렴 오차
                max_iteration=100,  # 최대 반복 횟수
            )
            reg_p2p = o3d.pipelines.registration.registration_icp(
                src_cloud,
                tgt_cloud,
                threshold,
                init_transform,
                o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                criteria,
            )
            draw_registration_result(src_cloud, tgt_cloud, reg_p2p.transformation)

            rmse = reg_p2p.inlier_rmse

            if rmse < 0.01:
                break
            
    # Save transformation matrix to YAML file
    yaml_path = "/home/nrs_vision/catkin_ws/src/nrs_ver2/config/params.yaml"
    save_to_yaml(yaml_path, "tf_matrix", reg_p2p.transformation.tolist())
    
    print(f"RMSE between transformed source and target clouds: {rmse:.6f}")

    print(reg_p2p.transformation)
    rospy.set_param("/tf_matrix", reg_p2p.transformation.tolist())
    rospy.set_param("/registration_done", True)


if __name__ == "__main__":
    main()
