#!/usr/bin/env python3
import rospy
import yaml
import os
import math
from tf.transformations import euler_from_matrix

def parse_tf_matrix(tf_matrix):
    """
    Parse a 1D or 2D tf_matrix from a YAML file into a 4x4 matrix.
    """
    if isinstance(tf_matrix[0], list):  # 2D array
        return tf_matrix
    elif isinstance(tf_matrix[0], (int, float)):  # 1D array
        return [
            [tf_matrix[0], tf_matrix[1], tf_matrix[2], tf_matrix[3]],
            [tf_matrix[4], tf_matrix[5], tf_matrix[6], tf_matrix[7]],
            [tf_matrix[8], tf_matrix[9], tf_matrix[10], tf_matrix[11]],
            [tf_matrix[12], tf_matrix[13], tf_matrix[14], tf_matrix[15]],
        ]
    else:
        rospy.logerr("Invalid tf_matrix format in YAML file.")
        return None

def update_xacro_file(xacro_file, translation, roll, pitch, yaw):
    """
    Update the scan_model.xacro file with new translation and rpy values.
    """
    if not os.path.isfile(xacro_file):
        rospy.logerr(f"Xacro file not found: {xacro_file}")
        return False

    with open(xacro_file, "r") as file:
        lines = file.readlines()

    updated_lines = []
    found_joint = False

    for line in lines:
        if '<joint name="cad_to_scan_model"' in line:
            found_joint = True
        if found_joint and '<origin ' in line:
            # Update the <origin> line
            updated_line = f'    <origin xyz="{translation[0]} {translation[1]} {translation[2]}" rpy="{roll} {pitch} {yaw}" />\n'
            updated_lines.append(updated_line)
            found_joint = False  # Stop updating after modifying the joint
        else:
            updated_lines.append(line)

    with open(xacro_file, "w") as file:
        file.writelines(updated_lines)

    rospy.loginfo(f"Updated {xacro_file} with translation={translation} and rpy={roll, pitch, yaw}")
    return True

def main():
    rospy.init_node("xacro_tf_node")

    # Load the YAML file
    yaml_file = rospy.get_param("~yaml_file")
    xacro_file = rospy.get_param("~xacro_file")
    if not os.path.isfile(yaml_file):
        rospy.logerr(f"YAML file not found: {yaml_file}")
        return

    with open(yaml_file, "r") as f:
        params = yaml.safe_load(f)

    # Extract the tf_matrix
    if "tf_matrix" not in params:
        rospy.logerr("tf_matrix not found in the YAML file.")
        return

    tf_matrix = parse_tf_matrix(params["tf_matrix"])
    if tf_matrix is None:
        return

    # Decompose the transformation matrix
    translation = [tf_matrix[0][3], tf_matrix[1][3], tf_matrix[2][3]]
    rotation_matrix = [
        [tf_matrix[0][0], tf_matrix[0][1], tf_matrix[0][2]],
        [tf_matrix[1][0], tf_matrix[1][1], tf_matrix[1][2]],
        [tf_matrix[2][0], tf_matrix[2][1], tf_matrix[2][2]],
    ]

    roll, pitch, yaw = euler_from_matrix(rotation_matrix, axes="sxyz")

    # Update the xacro file
    if update_xacro_file(xacro_file, translation, roll, pitch, yaw):
        rospy.set_param("/xacro_tf_done", True)

if __name__ == "__main__":
    main()
