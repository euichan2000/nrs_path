#!/usr/bin/env python3
import rospy
import open3d as o3d
import numpy as np
import os
import sys


def stl_to_pcd(stl_path):
    """
    STL 파일을 읽어와 레이캐스팅을 통해 PCD로 변환합니다.

    Parameters:
        stl_path (str): STL 파일의 경로.

    Returns:
        o3d.geometry.PointCloud: 변환된 포인트 클라우드.
    """
    print(f"Loading STL file from {stl_path}")
    mesh = o3d.io.read_triangle_mesh(stl_path)
    mesh.compute_triangle_normals()
    mesh_t = o3d.t.geometry.TriangleMesh.from_legacy(mesh)

    # Initialize a RaycastingScene and add the mesh
    scene = o3d.t.geometry.RaycastingScene()
    mesh_id = scene.add_triangles(mesh_t)

    # Define camera positions for raycasting
    camera_positions = {
        "top": ([0, 0, 5], [0, 0, 0], [0, 1, 0]),
        "front": ([0, -5, 0], [0, 0, 0], [0, 0, 1]),
        "back": ([0, 5, 0], [0, 0, 0], [0, 0, -1]),
        "left": ([-5, 0, 0], [0, 0, 0], [0, 1, 0]),
        "right": ([5, 0, 0], [0, 0, 0], [0, 1, 0]),
    }

    # Create an empty list to accumulate points
    all_points = []

    # Perform raycasting from each camera position
    for direction, (eye, center, up) in camera_positions.items():
        print(f"Raycasting from {direction} direction...")
        rays = o3d.t.geometry.RaycastingScene.create_rays_pinhole(
            fov_deg=90,
            center=center,
            eye=eye,
            up=up,
            width_px=4086,
            height_px=4086,
        )

        # Perform raycasting and extract valid hit points
        ans = scene.cast_rays(rays)
        hit = ans["t_hit"].isfinite()
        points = rays[hit][:, :3] + rays[hit][:, 3:] * ans["t_hit"][hit].reshape(
            (-1, 1)
        )
        all_points.append(points.numpy())
 
    # Combine all points into a single point cloud
    combined_points = np.vstack(all_points)

    pcd_combined = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(combined_points))

    # Perform voxel downsampling
    voxel_size = 0.003  # Adjust voxel size as needed
    pcd_downsampled = pcd_combined.voxel_down_sample(voxel_size)
    # Convert the GPU-based PointCloud to a CPU-based PointCloud
    
    return pcd_downsampled


def main():
    rospy.init_node("stl_to_pcd_node", anonymous=True)
    # ROS 파라미터에서 파일 경로 읽기
    stl_path = rospy.get_param("stl_to_pcd_node/source_file")
    if not stl_path:
        rospy.logerr(
            "STL or PCD file paths are not set. Please provide them via ROS parameters."
        )
        return
    # Generate output PCD file path by replacing .stl with .pcd
    pcd_path = rospy.get_param("stl_to_pcd_node/target_file")

    # Perform STL to PCD conversion
    pcd = stl_to_pcd(stl_path)

    # Save the PCD file
    print(f"Saving PCD file to {pcd_path}")
    
    o3d.io.write_point_cloud(
        pcd_path,
        pcd,
        write_ascii=True,
        compressed=False,
        print_progress=False,
    )
    print("Conversion complete!")
    rospy.set_param("/stl2pcd_done", True)


if __name__ == "__main__":
    main()
