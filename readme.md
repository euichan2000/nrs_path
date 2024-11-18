
# NRS_VISION_RVIZ Package

## Contents
0. [How to Use](#1-how-to-use)
1. [Dependencies](#1-dependencies)
2. [Launch](#2-launch)
3. [src](#3-src)
4. [urdf](#4-urdf)

## 0. How to Use
`roslaunch nrs_vision_rviz nrs_vision_rviz.launch`

1. Rviz 상단 `Publish Point` 버튼 클릭
2. Mesh 표면을 클릭하면서 Waypoints 생성
3. Terminal 창에 `straight` 혹은 `spline` 입력 시 path 생성
4. path 생성 후 자동으로 interpolation 실행
5. interpolation된 경로는 `/data/final_waypoints.txt`로 저장됨.
6. 해당 경로를 `rtde_handarm` 패키지로 옮겨서 실제 로봇에 사용

## 1. dependencies
### Rviz

```
sudo apt-get install ros-noetic-moveit
```

### PCL

```jsx
sudo apt-get update 
sudo apt-get install pcl-tools
```

### Moveit

```yaml
rosdep update

sudo apt update

sudo apt dist-upgrade

sudo apt install ros-noetic-catkin python3-catkin-tools

sudo apt install python3-wstool

cd ~/catkin_ws/src

wstool init .
wstool merge -t . https://raw.githubusercontent.com/ros-planning/moveit/master/moveit.rosinstall
wstool remove moveit_tutorials  # this is cloned in the next section
wstool update -t .

cd ~/catkin_ws/src
git clone https://github.com/ros-planning/moveit_tutorials.git -b master
git clone https://github.com/ros-planning/panda_moveit_config.git -b noetic-devel

rosdep install -y --from-paths . --ignore-src --rosdistro noetic

cd ..
catkin config --extend /opt/ros/noetic --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin_make
sou
source ~/catkin_ws/devel/setup.bash
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
```

### UR10

```jsx
sudo apt-get install ros-noetic-universal-robots
cd /path/to/catkin_ws/src
git clone -b noetic-devel https://github.com/ros-industrial/universal_robot.git
cd /path/to/catkin_ws
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro noetic
catkin_make
gedit ~/.bashrc
source /path/to/catkin_ws/devel/setup.bash
```

### CGAL

```jsx
sudo apt-get install libcgal-dev

wget https://github.com/CGAL/cgal/releases/download/v5.6.1/CGAL-5.6.1.tar.xz
tar xf CGAL-5.6.1.tar.xz
cd CGAL-5.6.1

mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4
sudo make install
```

### VTK

```jsx
sudo apt-get install libvtk7-dev
```

## 2. Launch
```
<launch>
  <!-- Launch RViz with a specified configuration file -->
  <node name="rviz" pkg="rviz" type="rviz" args="-d $(find nrs_vision_rviz)/config/rviz_config.rviz" />

  <!--mesh의 edge부분을 시각화하는 노드. mesh 자체는 URDF를 통해 로드. edge를 보고 싶지 않다면 주석처리-->
  <node name="visualize_mesh" pkg="nrs_vision_rviz" type="visualize_mesh" output="screen" /> 

  <!--Rviz를 통해 클릭한 점을 시각화하기 위한 노드-->
  <node name="point_publisher" pkg="nrs_vision_rviz" type="point_publisher" output="screen" />

  <!--clicked_point를 Subscribe하여 표면 위의 Geodesic 작업 경로를 생성하는 노드-->
  <node name="path_generator" pkg="nrs_vision_rviz" type="path_generator" output="screen" />
  <!--Rviz 상에 waypoints_interpolator를 통한 최종 경로를 시각화하기 위한 노드-->
  <node name="path_visualizer" pkg="nrs_vision_rviz" type="path_visualizer" output="screen" />

  <!--터미널 상에서 여러가지 input을 받고, 각 노드로 커맨드를 뿌려주기 위한 노드-->
  <node name="keyboard_listener" pkg="nrs_vision_rviz" type="keyboard_listener" output="screen" />

  <!--moveit을 이용해 rviz 상에서 시뮬레이션해서 시각화하기 위한 노드-->
  <node name="moveit_planner" pkg="nrs_vision_rviz" type="moveit_planner" output="screen" />

  <!--path_generator를 통해 생성된 작업 경로를 interpolation 및 normal smoothing, 접근,후퇴 경로 추가 등의 후처리하는 노드-->
  <node name="waypoints_interpolator" pkg="nrs_vision_rviz" type="waypoints_interpolator" output="screen" />



  <!-- 아래부터는 Moveit을 사용하기 위한 기본적인 설정들 -->

  <!-- Static transform publisher -->
  <node pkg="tf2_ros" type="static_transform_publisher" name="base_to_map"
    args="0 0 0 0 0 0 map base_link" />

  <!-- Joint state publisher -->
  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher"
    output="screen">
    <rosparam param="source_list">[move_group/fake_controller_joint_states]</rosparam>
  </node>

  <!-- Robot state publisher -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"
    output="screen">
    <!-- <param name="publish_frequency" type="double" value="50.0" /> -->
  </node>

  <!-- MoveIt! executable without trajectory execution -->
  <include file="$(find nrs_vision_rviz)/launch/move_group.launch">
    <arg name="allow_trajectory_execution" value="true" />
    <arg name="fake_execution" value="true" />
    <arg name="info" value="true" />
    <arg name="debug" value="false" />
    <arg name="pipeline" value="ompl" />
    <arg name="load_robot_description" value="true" />
  </include>


</launch>
  
```

## 3. Src
### 3.1. visualize_mesh.cpp
input으로 주어진 mesh의 edge를 시각화하기 위한 노드
### 3.2. path_generator.cpp

`path_generator.cpp` subscribes to the `nrs_command` topic to determine the type of path to generate:

- **Geodesic Spline**: If `use_spline` is true, it generates a geodesic spline passing through the defined waypoints.
- **Geodesic Straight Line**: If `use_straight` is true, it generates a geodesic straight line.

#### Adjustable Parameters:
- **Spline Density**:  
  Line 1247:  
  ```cpp
  steps = waypoints_distance * 50;
Increase the multiplier (e.g., 50) to create denser splines.
- **Curve Sharpness**:  
  Line 991:  
  ```cpp
  float c = 0.2;
**Range**: `0.0` to `1.0`
- **0.0**: Produces smoother curves.  
- **1.0**: Results in sharper, more straight-line-like paths.


### 3.3.waypoints_interpolator.cpp
#### **`generate_segment Function`**

#### Purpose:
Generates additional trajectory segments such as **approach**, **retreat**, and **home** based on the start or end of the provided waypoints.

#### Parameters:
- **original_points**: A vector of waypoints (start to end) for which segments will be generated.
- **option**: Integer specifying the type of segment to generate.
  - **1 (Approach Segment)**: Creates a segment moving towards the first waypoint along the normal vector of the mesh face it belongs to.
  - **2 (Retreat Segment)**: Creates a segment moving away from the last waypoint along the normal vector of the mesh face it belongs to.
  - **3 (Home Segment)**: Creates a segment leading the robot from the retreat point to a predefined home position.

#### Output:
A vector of waypoints representing the generated segment.

---

#### **`interpolatePoints Function`**

#### Purpose:
Interpolates waypoints to generate smoother trajectories with evenly spaced or variable intervals.

#### Parameters:
- **points**: Original waypoints to be interpolated.
- **desired_interval**: Target spacing between consecutive interpolated waypoints.
- **mesh**: Surface mesh used for geodesic distance calculation.
- **option**: Integer specifying the interpolation strategy.
  - **1 (Fixed Interval)**: Maintains a constant interval between waypoints.
  - **2 (Variable Interval)**: Adjusts spacing dynamically to create smooth transitions at the start and end of the trajectory.

#### Algorithm:
1. Computes cumulative geodesic distances between consecutive waypoints.
2. Interpolates new points at the desired interval or variable spacing.
3. Ensures continuity by interpolating within each segment.

#### Output:
A vector of interpolated waypoints.

---

#### **`convertToWaypoints Function`**

#### Purpose:
Transforms interpolated points into a waypoint message format (`nrs_vision_rviz/Waypoints`) while computing normals and orientations.

#### Parameters:
- **points**: Interpolated waypoints to be converted.
- **reference_points**: Reference points for calculating normal vectors and setting orientations.
- **mesh**: Surface mesh used for normal vector computation.
- **option**: Integer specifying the type of segment to process.
  - **1 (Approach Segment)**: Computes orientation based on the normal vector at the start point.
  - **2 (Original Segment)**: Interpolates normals along the mesh and assigns them to waypoints.
  - **3 (Retreat Segment)**: Computes orientation based on the normal vector at the end point.
  - **4 (Home Segment)**: Moves the robot to a predefined home position.
  - **5 (Face Normal)**: Directly uses face normals from the mesh to compute orientations.

#### Output:
A `nrs_vision_rviz/Waypoints` message with position and orientation data.

---

#### **`saveWaypointsToFile Function`**

#### Purpose:
Saves interpolated waypoints with positions, orientations, and forces to a text file for further use.

#### Parameters:
- **waypoints**: Waypoints message (`nrs_vision_rviz/Waypoints`) containing position and orientation data.
- **filename**: Path to the file where waypoints will be saved.
- **force**: A force value applied to all waypoints, useful for robot operations.

#### File Format:
Each line in the file represents a waypoint with the following fields:
- **Position (x, y, z)**: The 3D coordinates of the waypoint.
- **Orientation (roll, pitch, yaw)**: Orientation of the waypoint in RPY format.
- **Force**: A constant or variable force applied to the waypoint.

---

### Option Descriptions

#### **generate_segment Options**
| Option | Name              | Description                                                                 |
|--------|-------------------|-----------------------------------------------------------------------------|
| 1      | Approach Segment  | Moves towards the first waypoint along the normal vector of its mesh face. |
| 2      | Retreat Segment   | Moves away from the last waypoint along the normal vector of its mesh face. |
| 3      | Home Segment      | Moves the robot from the retreat point to a predefined home position.      |

#### **interpolatePoints Options**
| Option | Name              | Description                                                                                     |
|--------|-------------------|-------------------------------------------------------------------------------------------------|
| 1      | Fixed Interval    | Maintains a constant spacing between waypoints.                                                |
| 2      | Variable Interval | Adjusts spacing dynamically, with smoother transitions near the start and end of the trajectory.|

#### **convertToWaypoints Options**
| Option | Name              | Description                                                                                     |
|--------|-------------------|-------------------------------------------------------------------------------------------------|
| 1      | Approach Segment  | Computes orientation based on the normal vector at the start point.                            |
| 2      | Original Segment  | Computes interpolated normals along the mesh and assigns them to the waypoints.                |
| 3      | Retreat Segment   | Computes orientation based on the normal vector at the end point.                              |
| 4      | Home Segment      | Moves the robot to a predefined home position.                                                 |
| 5      | Face Normal       | Directly uses face normals from the mesh to compute orientations.                              |


### 3.4.path_visualizer.cpp
A node designed to visualize the path generated by the waypoints_interpolator node in Rviz.

### 3.5.moveit_planner.cpp
This node simulates and visualizes the execution of the path generated by the waypoints_interpolator node using the MoveIt! Planner with a UR10 or UR10e robot.

The following line defines the transform from the manipulator's end effector to the tool tip:


`tooltip_transform.setOrigin(tf2::Vector3(0.0, 0.0, -0.316));`

Here, the vector represents the translation of the TF matrix from the manipulator's end effector to the tool tip.
## 4. URDF
`testbed.xacro`: 
testbed, tooltip, workpiece의 stl Information & TF Definition 