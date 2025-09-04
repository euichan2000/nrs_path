#ifndef NRS_INTERPOLATION_H
#define NRS_INTERPOLATION_H

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "nrs_geodesic.h"

class nrs_interpolation
{
public:
    nrs_geodesic n_geodesic;
    // 헬퍼 함수: 원래 웨이포인트 간의 누적 거리를 계산
    std::vector<double> computeCumulativeDistances(const std::vector<geometry_msgs::Point> &points);

    // Option 1: 일정 간격 보간
    std::vector<geometry_msgs::Point> interpolatePoints_Constant(const std::vector<geometry_msgs::Point> &points,
                                                                 const std::vector<double> &cumulative_distances,
                                                                 double desired_interval);

    // Option 2: 가변 간격 보간
    std::vector<geometry_msgs::Point> interpolatePoints_Variable(const std::vector<geometry_msgs::Point> &points,
                                                                 const std::vector<double> &cumulative_distances,
                                                                 double desired_interval);

    // 주어진 점들을 원하는 간격으로 보간하는 함수 (option: 1 = 일정 간격, 2 = 가변 간격)
    std::vector<geometry_msgs::Point> interpolatePoints(const std::vector<geometry_msgs::Point> &points,
                                                        double desired_interval,
                                                        int option);

    // 옵션 1: Approach Segment
    // 시작점의 노멀(start_normal)을 기준으로 동일 orientation 적용
    nrs_path::Waypoints setToolVectorApproach(
        const std::vector<geometry_msgs::Point> &points,
        const Triangle_mesh &mesh,
        const Kernel::Vector_3 &start_normal);

    //--------------------------------------------------------
    // 옵션 2: Original Segment
    // 각 점마다 해당 점이 속한 face를 찾아, 바리센트릭 좌표를 이용해 보간된 노멀로 orientation 계산
    nrs_path::Waypoints setToolVectorOriginal(
        const std::vector<geometry_msgs::Point> &points,
        const Triangle_mesh &mesh,
        double Fx, double Fy, double Fz);

    //--------------------------------------------------------
    // 옵션 3: Retreat Segment
    // 마지막 reference point의 노멀(end_normal)을 기준으로 동일 orientation 적용
    nrs_path::Waypoints setToolVectorRetreat(
        const std::vector<geometry_msgs::Point> &points,
        const Triangle_mesh &mesh,
        const Kernel::Vector_3 &end_normal);

    //--------------------------------------------------------
    // 옵션 4: Home Segment
    // 마지막 reference point의 노멀(end_normal)을 기준으로 계산한 orientation으로 전체 구간에 동일 적용
    nrs_path::Waypoints setToolVectorHome(
        const std::vector<geometry_msgs::Point> &points,
        const Triangle_mesh &mesh,
        const Kernel::Vector_3 &end_normal);

    //--------------------------------------------------------

    nrs_path::Waypoints setToolVectorFaceNormal(
        const std::vector<geometry_msgs::Point> &points,
        const Triangle_mesh &mesh,
        double Fx, double Fy, double Fz);

    //--------------------------------------------------------
    // 최종 wrapper 함수: reference_points로부터 시작/끝 노멀 계산 후 옵션에 따라 헬퍼 호출
    nrs_path::Waypoints setToolVector(const std::vector<geometry_msgs::Point> &approach_interpolated,
                                      const std::vector<geometry_msgs::Point> &original_interpolated,
                                      const std::vector<geometry_msgs::Point> &retreat_interpolated,
                                      const std::vector<geometry_msgs::Point> &home_interpolated,
                                      const Triangle_mesh &mesh,
                                      double Fx, double Fy, double Fz);

    // 세그먼트(approach, retreat, home 등)를 생성하는 함수
    std::vector<geometry_msgs::Point> generate_segment(std::vector<geometry_msgs::Point> &original_points,
                                                       int option,
                                                       const Triangle_mesh &mesh);
    /// 간단한 가감속 프로파일링 함수
    // data: 입력 데이터 (예: 위치 값들의 배열)
    // startingTime: 시작 전 대기 시간 (초)
    // lastRestingTime: 이동 후 휴식 시간 (초)
    // accelerationTime: 가속/감속에 소요되는 시간 (초)
    // samplingTime: 샘플링 간격 (초)
    // 반환: 각 샘플 시점의 위치 값들을 담은 vector<double>

    // SLERP 헬퍼 함수: 두 tf2::Quaternion 사이의 보간 수행
    tf2::Quaternion quaternionSlerp(const tf2::Quaternion &q1, const tf2::Quaternion &q2, double t);
    void smoothQuaternions(nrs_path::Waypoints &wps, int window_size);
    // 쿼터니언 기반 보간 함수: 위치는 선형 보간, orientation은 SLERP를 사용하여 보간 후 Waypoints 메시지 생성
    nrs_path::Waypoints interpolateXYZQF(const nrs_path::Waypoints &input, double desired_interval);

    nrs_path::Waypoints interpolateEnd2End(const nrs_path::Waypoints &original_waypoints, double desired_interval, const Triangle_mesh &mesh, double Fx, double Fy, double Fz);

    Eigen::Vector3d getFaceNormal(const geometry_msgs::Point &ros_point, const Triangle_mesh &mesh);
};

#endif // NRS_INTERPOLATION_H
