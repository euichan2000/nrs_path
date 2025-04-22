import rospy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def main():
    rospy.init_node('path_alignment_node')
    
    # Load the original path; assuming the file format is (N, 3)
    old_path = np.loadtxt("/home/nrs/catkin_ws/src/nrs_path/data/Ori_path.txt", delimiter=" ")
    
    # Compute the center of the original path
    old_center = np.mean(old_path, axis=0)
    
    # Load the new center from file
    new_center_file = "/home/nrs/catkin_ws/src/nrs_path/data/Mesh_center.txt"
    try:
        new_center = np.loadtxt(new_center_file, delimiter=" ")
    except Exception as e:
        rospy.logerr(f"Failed to load new center from {new_center_file}: {e}")
        return

    # Apply translation to align the old path to the new center
    aligned_path = old_path - old_center + new_center
    
    # Save the transformed path to a file
    transformed_path_file = "/home/nrs/catkin_ws/src/nrs_path/data/Ori_path_transformed.txt"
    np.savetxt(transformed_path_file, aligned_path, fmt="%.6f")
    rospy.loginfo(f"Transformed path saved to {transformed_path_file}")
    
    # Visualization
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(*old_path.T, label='Original Path', color='blue', marker='o')
    ax.plot(*aligned_path.T, label='Translated Path', color='green', marker='^')
    ax.scatter(*new_center, color='red', label='New Center (File)', s=50)

    ax.legend()
    ax.set_title("Path Aligned using Mesh Center")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.view_init(elev=30, azim=45)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
