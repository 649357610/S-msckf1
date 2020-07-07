/*
 * COPYRIGHT AND PERMISSION NOTICE
 * Penn Software MSCKF_VIO
 * Copyright (C) 2017 The Trustees of the University of Pennsylvania
 * All rights reserved.
 */

#ifndef MSCKF_VIO_FEATURE_H
#define MSCKF_VIO_FEATURE_H

#include <iostream>
#include <map>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

#include "math_utils.hpp"
#include "imu_state.h"
#include "cam_state.h"

namespace msckf_vio {

/*
 * @brief Feature Salient part of an image. Please refer
 *    to the Appendix of "A Multi-State Constraint Kalman
 *    Filter for Vision-aided Inertial Navigation" for how
 *    the 3d position of a feature is initialized.
 */
//用于msckf_vio，而非imageprocessor
struct Feature {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  typedef long long int FeatureIDType;

  /*
   * @brief OptimizationConfig Configuration parameters
   *    for 3d feature position optimization.
   */
  //优化参数
  struct OptimizationConfig {
    double translation_threshold;  //用于判断相机的运动是否足够大，用于checkMotion()函数
    double huber_epsilon;
    double estimation_precision;
    double initial_damping;
    int outer_loop_max_iteration;
    int inner_loop_max_iteration;

    OptimizationConfig():
      translation_threshold(0.2),
      huber_epsilon(0.01),
      estimation_precision(5e-7),
      initial_damping(1e-3),
      outer_loop_max_iteration(10),
      inner_loop_max_iteration(10) {
      return;
    }
  };

  // Constructors for the struct.
  Feature(): id(0), position(Eigen::Vector3d::Zero()),
    is_initialized(false) {}

  Feature(const FeatureIDType& new_id): id(new_id),
    position(Eigen::Vector3d::Zero()),
    is_initialized(false) {}

  /*
   * @brief cost Compute the cost of the camera observations
   * @param T_c0_c1 A rigid body transformation takes
   *    a vector in c0 frame to ci frame.
   * @param x The current estimation.
   * @param z The ith measurement of the feature j in ci frame.
   * @return e The cost of this observation.
   */
  inline void cost(const Eigen::Isometry3d& T_c0_ci,
      const Eigen::Vector3d& x, const Eigen::Vector2d& z,
      double& e) const;

  /*
   * @brief jacobian Compute the Jacobian of the camera observation
   * @param T_c0_c1 A rigid body transformation takes
   *    a vector in c0 frame to ci frame.
   * @param x The current estimation.
   * @param z The actual measurement of the feature in ci frame.
   * @return J The computed Jacobian.
   * @return r The computed residual.
   * @return w Weight induced by huber kernel.
   */
  inline void jacobian(const Eigen::Isometry3d& T_c0_ci,
      const Eigen::Vector3d& x, const Eigen::Vector2d& z,
      Eigen::Matrix<double, 2, 3>& J, Eigen::Vector2d& r,
      double& w) const;

  /*
   * @brief generateInitialGuess Compute the initial guess of
   *    the feature's 3d position using only two views.
   * @param T_c1_c2: A rigid body transformation taking
   *    a vector from c2 frame to c1 frame.
   * @param z1: feature observation in c1 frame.
   * @param z2: feature observation in c2 frame.
   * @return p: Computed feature position in c1 frame.
   */
  inline void generateInitialGuess(
      const Eigen::Isometry3d& T_c1_c2, const Eigen::Vector2d& z1,
      const Eigen::Vector2d& z2, Eigen::Vector3d& p) const;

  /*
   * @brief checkMotion Check the input camera poses to ensure
   *    there is enough translation to triangulate the feature
   *    positon.
   * @param cam_states : input camera poses.
   * @return True if the translation between the input camera
   *    poses is sufficient.
   */
  inline bool checkMotion(
      const CamStateServer& cam_states) const;

  /*
   * @brief InitializePosition Intialize the feature position
   *    based on all current available measurements.
   * @param cam_states: A map containing the camera poses with its
   *    ID as the associated key value.
   * @return The computed 3d position is used to set the position
   *    member variable. Note the resulted position is in world
   *    frame.
   * @return True if the estimated 3d position of the feature
   *    is valid.
   */
  inline bool initializePosition(
      const CamStateServer& cam_states);


  // An unique identifier for the feature.
  // In case of long time running, the variable
  // type of id is set to FeatureIDType in order
  // to avoid duplication.
  FeatureIDType id;

  // id for next feature
  static FeatureIDType next_id;

  // Store the observations of the features in the
  // state_id(key)-image_coordinates(value) manner.
  //key表示此feature所在的图像id，value表示此feature所在此图像中的左右目归一化坐标
  std::map<StateIDType, Eigen::Vector4d, std::less<StateIDType>,
    Eigen::aligned_allocator<
      std::pair<const StateIDType, Eigen::Vector4d> > > observations;

  // 3d postion of the feature in the world frame.
  Eigen::Vector3d position;

  // A indicator to show if the 3d postion of the feature
  // has been initialized or not.
  bool is_initialized;

  // Noise for a normalized feature measurement.
  static double observation_noise;

  // Optimization configuration for solving the 3d position.
  static OptimizationConfig optimization_config;

};

typedef Feature::FeatureIDType FeatureIDType;
//所有的future的信息组合在一起，形成MapServer
typedef std::map<FeatureIDType, Feature, std::less<int>,
        Eigen::aligned_allocator<
        std::pair<const FeatureIDType, Feature> > > MapServer;

//计算观测残差的平方和
//输入：ci到c0的变换矩阵T_c0_ci
//输入：三角化后得到的特征点的3d世界坐标x
//输入：2d观测值z（归一化相机平面坐标）
//输出：残差的平方和e
void Feature::cost(const Eigen::Isometry3d& T_c0_ci,
    const Eigen::Vector3d& x, const Eigen::Vector2d& z,
    double& e) const {
  // Compute hi1, hi2, and hi3 as Equation (37).
  const double& alpha = x(0);
  const double& beta = x(1);
  const double& rho = x(2);

  Eigen::Vector3d h = T_c0_ci.linear()*
    Eigen::Vector3d(alpha, beta, 1.0) + rho*T_c0_ci.translation();
  double& h1 = h(0);
  double& h2 = h(1);
  double& h3 = h(2);

  // Predict the feature observation in ci frame.
  Eigen::Vector2d z_hat(h1/h3, h2/h3);

  // Compute the residual.
  //向量的平方范数由squaredNorm()获得，等价于向量对自身做点积，也等同于所有元素平方和。
  e = (z_hat-z).squaredNorm();
  return;
}
//计算观测残差对alpha，beta，rho的雅可比，用于三角化计算最小二乘
//输入：c0到ci的变换矩阵T_c0_ci
//输入：待估计的特征点的c0系下的3d逆深度坐标x
//输入：2d观测值z（归一化相机平面坐标）
//输出：观测残差对alpha，beta，rho的雅可比J
//输出：观测残差r，注意：残差定义为r = z_hat - z;估计值-观测值
//输出：根据残差获得huber权重
void Feature::jacobian(const Eigen::Isometry3d& T_c0_ci,
    const Eigen::Vector3d& x, const Eigen::Vector2d& z,
    Eigen::Matrix<double, 2, 3>& J, Eigen::Vector2d& r,
    double& w) const {

  // Compute hi1, hi2, and hi3 as Equation (37).
  const double& alpha = x(0);
  const double& beta = x(1);
  const double& rho = x(2);

  Eigen::Vector3d h = T_c0_ci.linear()*
    Eigen::Vector3d(alpha, beta, 1.0) + rho*T_c0_ci.translation();
  double& h1 = h(0);
  double& h2 = h(1);
  double& h3 = h(2);

  // Compute the Jacobian.
  Eigen::Matrix3d W;
  W.leftCols<2>() = T_c0_ci.linear().leftCols<2>();
  W.rightCols<1>() = T_c0_ci.translation();

  J.row(0) = 1/h3*W.row(0) - h1/(h3*h3)*W.row(2);
  J.row(1) = 1/h3*W.row(1) - h2/(h3*h3)*W.row(2);

  // Compute the residual.
  Eigen::Vector2d z_hat(h1/h3, h2/h3);
  r = z_hat - z;

  // Compute the weight based on the residual.
  double e = r.norm();
  if (e <= optimization_config.huber_epsilon)
    w = 1.0;
  else
    w = std::sqrt(2.0*optimization_config.huber_epsilon / e);

  return;
}
//特征点三角化分两步，第一步是根据对特征点的某两帧观测值对其深度进行初始估计，即此函数的作用，当作三角化的初值
//使用两帧图像进行线性最小二乘估计特征点初始深度
//输入：c1到c2的变换
//输入：像素坐标z1
//输入：像素坐标z2
//输出：特征点在c1坐标系下的3d坐标
void Feature::generateInitialGuess(
    const Eigen::Isometry3d& T_c1_c2, const Eigen::Vector2d& z1,
    const Eigen::Vector2d& z2, Eigen::Vector3d& p) const {
  // Construct a least square problem to solve the depth.
  Eigen::Vector3d m = T_c1_c2.linear() * Eigen::Vector3d(z1(0), z1(1), 1.0);

  Eigen::Vector2d A(0.0, 0.0);
  A(0) = m(0) - z2(0)*m(2);
  A(1) = m(1) - z2(1)*m(2);

  Eigen::Vector2d b(0.0, 0.0);
  b(0) = z2(0)*T_c1_c2.translation()(2) - T_c1_c2.translation()(0);
  b(1) = z2(1)*T_c1_c2.translation()(2) - T_c1_c2.translation()(1);

  // Solve for the depth.
  //线性最小二乘(A'A)^-1*Ab
  double depth = (A.transpose() * A).inverse() * A.transpose() * b;
  p(0) = z1(0) * depth;
  p(1) = z1(1) * depth;
  p(2) = depth;
  return;
}
//通过计算该特征点的所有观测帧中第一帧与最后一帧的视差来判断是否有足够大的运动
//视差足够大，才可以三角化
//输入：观测到该特征的所有cam状态
//输出：如果有足够大的运动，返回1,否则返回0
bool Feature::checkMotion(
    const CamStateServer& cam_states) const {

  const StateIDType& first_cam_id = observations.begin()->first;
  const StateIDType& last_cam_id = (--observations.end())->first;

  //构造的是T_c_w,表示向量从c系到w系的变换矩阵
  Eigen::Isometry3d first_cam_pose;
  //R_c_w
  first_cam_pose.linear() = quaternionToRotation(
      cam_states.find(first_cam_id)->second.orientation).transpose();
  //p_c_w
  first_cam_pose.translation() =
    cam_states.find(first_cam_id)->second.position;

  //构造的是T_c_w,表示向量从c系到w系的变换矩阵
  Eigen::Isometry3d last_cam_pose;
  last_cam_pose.linear() = quaternionToRotation(
      cam_states.find(last_cam_id)->second.orientation).transpose();
  last_cam_pose.translation() =
    cam_states.find(last_cam_id)->second.position;

  // Get the direction of the feature when it is first observed.
  // This direction is represented in the world frame.
  //将特征点的方向（单位向量）（c系下P_cp）转到世界坐标系（w系下P_cp）
  Eigen::Vector3d feature_direction(
      observations.begin()->second(0),
      observations.begin()->second(1), 1.0);
  feature_direction = feature_direction / feature_direction.norm();
  feature_direction = first_cam_pose.linear()*feature_direction;

  // Compute the translation between the first frame
  // and the last frame. We assume the first frame and
  // the last frame will provide the largest motion to
  // speed up the checking process.
  // 在判断是否能三角化时,用的是第一个观测和最后一个观测
  Eigen::Vector3d translation = last_cam_pose.translation() -
    first_cam_pose.translation();
  //todo 没看懂，为什么这样算视差？
  //相当于 translation点乘feature_direction，feature_direction是单位向量，所以|translation|*cos(theta)
  double parallel_translation =
    translation.transpose()*feature_direction;
  Eigen::Vector3d orthogonal_translation = translation -
    parallel_translation*feature_direction;

  if (orthogonal_translation.norm() >
      optimization_config.translation_threshold)
    return true;
  else return false;
}
//对特征点逆深度坐标用LM方法进行三角化（多个约束）
//输入：观测到该特征点的相机cam_states
//输出：是否初始化成功（特征点在每个相机前面）
//输出：三角化后的特征点在世界坐标系下的3d坐标（类成员变量）
bool Feature::initializePosition(
    const CamStateServer& cam_states) {
  // Organize camera poses and feature observations properly.
  std::vector<Eigen::Isometry3d,
    Eigen::aligned_allocator<Eigen::Isometry3d> > cam_poses(0);
  std::vector<Eigen::Vector2d,
    Eigen::aligned_allocator<Eigen::Vector2d> > measurements(0);

  for (auto& m : observations) {
    // TODO: This should be handled properly. Normally, the
    //    required camera states should all be available in
    //    the input cam_states buffer.
    auto cam_state_iter = cam_states.find(m.first);
    if (cam_state_iter == cam_states.end()) continue;

    // Add the measurement.
    measurements.push_back(m.second.head<2>());
    measurements.push_back(m.second.tail<2>());

    // This camera pose will take a vector from this camera frame
    // to the world frame.
    Eigen::Isometry3d cam0_pose;
    cam0_pose.linear() = quaternionToRotation(
        cam_state_iter->second.orientation).transpose();
    cam0_pose.translation() = cam_state_iter->second.position;

    Eigen::Isometry3d cam1_pose;
    cam1_pose = cam0_pose * CAMState::T_cam0_cam1.inverse();

    cam_poses.push_back(cam0_pose);
    cam_poses.push_back(cam1_pose);
  }

  // All camera poses should be modified such that it takes a
  // vector from the first camera frame in the buffer to this
  // camera frame.
  Eigen::Isometry3d T_c0_w = cam_poses[0];
  for (auto& pose : cam_poses)
    pose = pose.inverse() * T_c0_w;

  // Generate initial guess
  Eigen::Vector3d initial_position(0.0, 0.0, 0.0);
  generateInitialGuess(cam_poses[cam_poses.size()-1], measurements[0],
      measurements[measurements.size()-1], initial_position);
  //逆深度表示
  Eigen::Vector3d solution(
      initial_position(0)/initial_position(2),
      initial_position(1)/initial_position(2),
      1.0/initial_position(2));

  // Apply Levenberg-Marquart method to solve for the 3d position.
  double lambda = optimization_config.initial_damping;
  int inner_loop_cntr = 0;
  int outer_loop_cntr = 0;
  bool is_cost_reduced = false;
  double delta_norm = 0;

  // Compute the initial cost.
  double total_cost = 0.0;
  for (int i = 0; i < cam_poses.size(); ++i) {
    double this_cost = 0.0;
    cost(cam_poses[i], solution, measurements[i], this_cost);
    total_cost += this_cost;
  }
  //使用LM算法，对特征点三角化
  // Outer loop.
  do {
    Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
    Eigen::Vector3d b = Eigen::Vector3d::Zero();

    for (int i = 0; i < cam_poses.size(); ++i) {
      Eigen::Matrix<double, 2, 3> J;
      Eigen::Vector2d r;
      double w;

      jacobian(cam_poses[i], solution, measurements[i], J, r, w);

      if (w == 1) {
        A += J.transpose() * J;
        b += J.transpose() * r;
      } else {
        double w_square = w * w;
        A += w_square * J.transpose() * J;
        b += w_square * J.transpose() * r;
      }
    }

    // Inner loop.
    // Solve for the delta that can reduce the total cost.
    do {
      Eigen::Matrix3d damper = lambda * Eigen::Matrix3d::Identity();
      Eigen::Vector3d delta = (A+damper).ldlt().solve(b);
      Eigen::Vector3d new_solution = solution - delta;
      delta_norm = delta.norm();

      double new_cost = 0.0;
      for (int i = 0; i < cam_poses.size(); ++i) {
        double this_cost = 0.0;
        cost(cam_poses[i], new_solution, measurements[i], this_cost);
        new_cost += this_cost;
      }

      if (new_cost < total_cost) {
        is_cost_reduced = true;
        solution = new_solution;
        total_cost = new_cost;
        lambda = lambda/10 > 1e-10 ? lambda/10 : 1e-10;
      } else {
        is_cost_reduced = false;
        lambda = lambda*10 < 1e12 ? lambda*10 : 1e12;
      }

    } while (inner_loop_cntr++ <
        optimization_config.inner_loop_max_iteration && !is_cost_reduced);

    inner_loop_cntr = 0;

  } while (outer_loop_cntr++ <
      optimization_config.outer_loop_max_iteration &&
      delta_norm > optimization_config.estimation_precision);

  // Covert the feature position from inverse depth
  // representation to its 3d coordinate.
  Eigen::Vector3d final_position(solution(0)/solution(2),
      solution(1)/solution(2), 1.0/solution(2));

  // Check if the solution is valid. Make sure the feature
  // is in front of every camera frame observing it.
  bool is_valid_solution = true;
  for (const auto& pose : cam_poses) {
      //将特征点坐标变换到每个相机坐标系下
    Eigen::Vector3d position =
      pose.linear()*final_position + pose.translation();
    if (position(2) <= 0) {
      is_valid_solution = false;
      break;
    }
  }
  //原来的特征点是在c0系下
  // Convert the feature position to the world frame.
  position = T_c0_w.linear()*final_position + T_c0_w.translation();

  if (is_valid_solution)
    is_initialized = true;

  return is_valid_solution;
}
} // namespace msckf_vio

#endif // MSCKF_VIO_FEATURE_H
