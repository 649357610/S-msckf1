/*
 * COPYRIGHT AND PERMISSION NOTICE
 * Penn Software MSCKF_VIO
 * Copyright (C) 2017 The Trustees of the University of Pennsylvania
 * All rights reserved.
 */

#include <iostream>
#include <iomanip>
#include <cmath>
#include <iterator>
#include <algorithm>

#include <Eigen/SVD>
#include <Eigen/QR>
#include <Eigen/SparseCore>
#include <Eigen/SPQRSupport>
#include <boost/math/distributions/chi_squared.hpp>

#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <msckf_vio/msckf_vio.h>
#include <msckf_vio/math_utils.hpp>
#include <msckf_vio/utils.h>

using namespace std;
using namespace Eigen;

namespace msckf_vio{
//静态成员对象，值可以被修改
//https://www.cnblogs.com/songhe364826110/p/11546148.html
//https://blog.csdn.net/dengfan666/article/details/78618289
// Static member variables in IMUState class.
StateIDType IMUState::next_id = 0;
//IMU噪声的初始协方差，标定得来
double IMUState::gyro_noise = 0.001;
double IMUState::acc_noise = 0.01;
double IMUState::gyro_bias_noise = 0.001;
double IMUState::acc_bias_noise = 0.01;
Vector3d IMUState::gravity = Vector3d(0, 0, -GRAVITY_ACCELERATION);//world系下的重力
//表示将向量从imu系转换到body系的变换矩阵
Isometry3d IMUState::T_imu_body = Isometry3d::Identity();

// Static member variables in CAMState class.
//takes a vector from the cam0 frame to the cam1 frame.
Isometry3d CAMState::T_cam0_cam1 = Isometry3d::Identity();

// Static member variables in Feature class.
FeatureIDType Feature::next_id = 0;
double Feature::observation_noise = 0.01;
Feature::OptimizationConfig Feature::optimization_config;
//https://www.cnblogs.com/ceason/articles/12852889.html
map<int, double> MsckfVio::chi_squared_test_table;

MsckfVio::MsckfVio(ros::NodeHandle& pnh):
  is_gravity_set(false),
  is_first_img(true),
  nh(pnh) {
  return;
}
//从ros服务器获取相关参数
bool MsckfVio::loadParameters() {
  // Frame id
  nh.param<string>("fixed_frame_id", fixed_frame_id, "world");
  nh.param<string>("child_frame_id", child_frame_id, "robot");
  nh.param<bool>("publish_tf", publish_tf, true);
  nh.param<double>("frame_rate", frame_rate, 40.0);
  nh.param<double>("position_std_threshold", position_std_threshold, 8.0);

  nh.param<double>("rotation_threshold", rotation_threshold, 0.2618);
  nh.param<double>("translation_threshold", translation_threshold, 0.4);
  nh.param<double>("tracking_rate_threshold", tracking_rate_threshold, 0.5);

  // Feature optimization parameters
  nh.param<double>("feature/config/translation_threshold",
      Feature::optimization_config.translation_threshold, 0.2);

  // Noise related parameters
  nh.param<double>("noise/gyro", IMUState::gyro_noise, 0.001);
  nh.param<double>("noise/acc", IMUState::acc_noise, 0.01);
  nh.param<double>("noise/gyro_bias", IMUState::gyro_bias_noise, 0.001);
  nh.param<double>("noise/acc_bias", IMUState::acc_bias_noise, 0.01);
  nh.param<double>("noise/feature", Feature::observation_noise, 0.01);

  // Use variance instead of standard deviation.
  //从ros中获取的是标准差，执行完下面这步gyro_noise中存为方差
  IMUState::gyro_noise *= IMUState::gyro_noise;
  IMUState::acc_noise *= IMUState::acc_noise;
  IMUState::gyro_bias_noise *= IMUState::gyro_bias_noise;
  IMUState::acc_bias_noise *= IMUState::acc_bias_noise;
  Feature::observation_noise *= Feature::observation_noise;

  // Set the initial IMU state.
  // The intial orientation and position will be set to the origin
  // implicitly. But the initial velocity and bias can be
  // set by parameters.
  // TODO: is it reasonable to set the initial bias to 0?
  nh.param<double>("initial_state/velocity/x",
      state_server.imu_state.velocity(0), 0.0);
  nh.param<double>("initial_state/velocity/y",
      state_server.imu_state.velocity(1), 0.0);
  nh.param<double>("initial_state/velocity/z",
      state_server.imu_state.velocity(2), 0.0);

  // The initial covariance of orientation and position can be
  // set to 0. But for velocity, bias and extrinsic parameters,
  // there should be nontrivial uncertainty.
  double gyro_bias_cov, acc_bias_cov, velocity_cov;
  nh.param<double>("initial_covariance/velocity",
      velocity_cov, 0.25);
  nh.param<double>("initial_covariance/gyro_bias",
      gyro_bias_cov, 1e-4);
  nh.param<double>("initial_covariance/acc_bias",
      acc_bias_cov, 1e-2);

  double extrinsic_rotation_cov, extrinsic_translation_cov;
  nh.param<double>("initial_covariance/extrinsic_rotation_cov",
      extrinsic_rotation_cov, 3.0462e-4);
  nh.param<double>("initial_covariance/extrinsic_translation_cov",
      extrinsic_translation_cov, 1e-4);

  state_server.state_cov = MatrixXd::Zero(21, 21);
  for (int i = 3; i < 6; ++i)
    state_server.state_cov(i, i) = gyro_bias_cov;
  for (int i = 6; i < 9; ++i)
    state_server.state_cov(i, i) = velocity_cov;
  for (int i = 9; i < 12; ++i)
    state_server.state_cov(i, i) = acc_bias_cov;
  for (int i = 15; i < 18; ++i)
    state_server.state_cov(i, i) = extrinsic_rotation_cov;
  for (int i = 18; i < 21; ++i)
    state_server.state_cov(i, i) = extrinsic_translation_cov;
  //注意config文件中T_cam_imu表示向量送imu系到cam的变换，与程序中的表示正好相反
  // Transformation offsets between the frames involved.
  Isometry3d T_imu_cam0 = utils::getTransformEigen(nh, "cam0/T_cam_imu");
  //Isometry3d T_cam0_imu求逆，得到T_imu_cam0，不是普通的求逆
  Isometry3d T_cam0_imu = T_imu_cam0.inverse();

  state_server.imu_state.R_imu_cam0 = T_cam0_imu.linear().transpose();
  state_server.imu_state.t_cam0_imu = T_cam0_imu.translation();
  CAMState::T_cam0_cam1 =
    utils::getTransformEigen(nh, "cam1/T_cn_cnm1");
  IMUState::T_imu_body =
    utils::getTransformEigen(nh, "T_imu_body").inverse();

  // Maximum number of camera states to be stored
  nh.param<int>("max_cam_state_size", max_cam_state_size, 30);

  ROS_INFO("===========================================");
  ROS_INFO("fixed frame id: %s", fixed_frame_id.c_str());
  ROS_INFO("child frame id: %s", child_frame_id.c_str());
  ROS_INFO("publish tf: %d", publish_tf);
  ROS_INFO("frame rate: %f", frame_rate);
  ROS_INFO("position std threshold: %f", position_std_threshold);
  ROS_INFO("Keyframe rotation threshold: %f", rotation_threshold);
  ROS_INFO("Keyframe translation threshold: %f", translation_threshold);
  ROS_INFO("Keyframe tracking rate threshold: %f", tracking_rate_threshold);
  ROS_INFO("gyro noise: %.10f", IMUState::gyro_noise);
  ROS_INFO("gyro bias noise: %.10f", IMUState::gyro_bias_noise);
  ROS_INFO("acc noise: %.10f", IMUState::acc_noise);
  ROS_INFO("acc bias noise: %.10f", IMUState::acc_bias_noise);
  ROS_INFO("observation noise: %.10f", Feature::observation_noise);
  ROS_INFO("initial velocity: %f, %f, %f",
      state_server.imu_state.velocity(0),
      state_server.imu_state.velocity(1),
      state_server.imu_state.velocity(2));
  ROS_INFO("initial gyro bias cov: %f", gyro_bias_cov);
  ROS_INFO("initial acc bias cov: %f", acc_bias_cov);
  ROS_INFO("initial velocity cov: %f", velocity_cov);
  ROS_INFO("initial extrinsic rotation cov: %f",
      extrinsic_rotation_cov);
  ROS_INFO("initial extrinsic translation cov: %f",
      extrinsic_translation_cov);

  cout << T_imu_cam0.linear() << endl;
  cout << T_imu_cam0.translation().transpose() << endl;

  ROS_INFO("max camera state #: %d", max_cam_state_size);
  ROS_INFO("===========================================");
  return true;
}
//创建ros io
//发布话题，接收话题
bool MsckfVio::createRosIO() {
  odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 10);
  feature_pub = nh.advertise<sensor_msgs::PointCloud2>(
      "feature_point_cloud", 10);

  reset_srv = nh.advertiseService("reset",
      &MsckfVio::resetCallback, this);

  imu_sub = nh.subscribe("imu", 100,
      &MsckfVio::imuCallback, this);
  feature_sub = nh.subscribe("features", 40,
      &MsckfVio::featureCallback, this);

  mocap_odom_sub = nh.subscribe("mocap_odom", 10,
      &MsckfVio::mocapOdomCallback, this);
  mocap_odom_pub = nh.advertise<nav_msgs::Odometry>("gt_odom", 1);

  return true;
}
/*1.判断是否初始化成功
 *2.Initialize state server
 *3.创建卡方检测表
 *
 * 初始化成功条件：1.loadParameters()完成，  2.createRosIO()完成
 */
bool MsckfVio::initialize() {
  if (!loadParameters()) return false;
  ROS_INFO("Finish loading ROS parameters...");

  // Initialize state server
  //初始化连续时间噪声
  state_server.continuous_noise_cov =
    Matrix<double, 12, 12>::Zero();
  state_server.continuous_noise_cov.block<3, 3>(0, 0) =
    Matrix3d::Identity()*IMUState::gyro_noise;
  state_server.continuous_noise_cov.block<3, 3>(3, 3) =
    Matrix3d::Identity()*IMUState::gyro_bias_noise;
  state_server.continuous_noise_cov.block<3, 3>(6, 6) =
    Matrix3d::Identity()*IMUState::acc_noise;
  state_server.continuous_noise_cov.block<3, 3>(9, 9) =
    Matrix3d::Identity()*IMUState::acc_bias_noise;

  // Initialize the chi squared test table with confidence
  // 建立卡方表level 0.95.
  //todo 函数怎么用？
  for (int i = 1; i < 100; ++i) {
    boost::math::chi_squared chi_squared_dist(i);
    chi_squared_test_table[i] =
      boost::math::quantile(chi_squared_dist, 0.05);
  }

  if (!createRosIO()) return false;
  ROS_INFO("Finish creating ROS IO...");

  return true;
}
/*1.将imu数据存储到imu_msg_buffer中
 *2.如果重力向量未初始化，则imu_msg_buffer存储了200个数据时，对重力向量和bias进行估计
 *  即调用initializeGravityAndBias()
 */
void MsckfVio::imuCallback(
    const sensor_msgs::ImuConstPtr& msg) {

  // IMU msgs are pushed backed into a buffer instead of
  // being processed immediately. The IMU msgs are processed
  // when the next image is available, in which way, we can
  // easily handle the transfer delay.
  imu_msg_buffer.push_back(*msg);
//如果重力没有初始化，则使用前200个imu数据估计重力向量
  if (!is_gravity_set) {
    if (imu_msg_buffer.size() < 200) return;
    //if (imu_msg_buffer.size() < 10) return;
    initializeGravityAndBias();
    is_gravity_set = true;
  }

  return;
}
/*
 *step1.利用前200个imu数据估计imu系下的重力向量和陀螺仪bias
 *step2.根据imu系下重力向量估计world系下的重力向量
 *step3.根据imu系和world系下的重力向量估计imu到world的旋转
 *step4.更新相关成员变量IMUState::gravity和state_server.imu_state.orientation
 *
 * 静止状态下利用前200个imu数据， 静止状态下陀螺仪的均值就是bias在imu系下的估计值，
 * 静止状态下加速度计的均值就是重力在imu系下的估计值，所以初始化一定要在系统静止时进行
 * 否则会初始化失败，前端能匹配成功，但后端没有里程计输出的。
 */
void MsckfVio::initializeGravityAndBias() {

  // Initialize gravity and gyro bias.
  Vector3d sum_angular_vel = Vector3d::Zero();
  Vector3d sum_linear_acc = Vector3d::Zero();
//step1.利用前200个imu数据估计imu系下的重力向量和陀螺仪bias
  for (const auto& imu_msg : imu_msg_buffer) {
    Vector3d angular_vel = Vector3d::Zero();
    Vector3d linear_acc = Vector3d::Zero();
  //https://blog.csdn.net/CSDNhuaong/article/details/83050860
  //ROS的消息类型转为eigen类型
    tf::vectorMsgToEigen(imu_msg.angular_velocity, angular_vel);
    tf::vectorMsgToEigen(imu_msg.linear_acceleration, linear_acc);

    sum_angular_vel += angular_vel;
    sum_linear_acc += linear_acc;
  }
//静止状态下陀螺仪的均值就是bias在imu系下的估计值
  state_server.imu_state.gyro_bias =
    sum_angular_vel / imu_msg_buffer.size();
  //IMUState::gravity =
  //  -sum_linear_acc / imu_msg_buffer.size();
  // This is the gravity in the IMU frame.
// 静止状态下加速度计的均值就是重力在imu系下的估计值
  Vector3d gravity_imu =
    sum_linear_acc / imu_msg_buffer.size();

  // Initialize the initial orientation, so that the estimation
  // is consistent with the inertial frame.
  double gravity_norm = gravity_imu.norm();
//step2.根据imu系下重力向量估计world系下的重力向量
  IMUState::gravity = Vector3d(0.0, 0.0, -gravity_norm);
//返回q_i_w
//step3.根据imu系和world系下的重力向量估计imu到world的旋转
  Quaterniond q0_i_w = Quaterniond::FromTwoVectors(
    gravity_imu, -IMUState::gravity);
//step4.更新相关成员变量IMUState::gravity和state_server.imu_state.orientation
  state_server.imu_state.orientation =
    rotationToQuaternion(q0_i_w.toRotationMatrix().transpose());

  return;
}

bool MsckfVio::resetCallback(
    std_srvs::Trigger::Request& req,
    std_srvs::Trigger::Response& res) {

  ROS_WARN("Start resetting msckf vio...");
  // Temporarily shutdown the subscribers to prevent the
  // state from updating.
  feature_sub.shutdown();
  imu_sub.shutdown();

  // Reset the IMU state.
  IMUState& imu_state = state_server.imu_state;
  imu_state.time = 0.0;
  imu_state.orientation = Vector4d(0.0, 0.0, 0.0, 1.0);
  imu_state.position = Vector3d::Zero();
  imu_state.velocity = Vector3d::Zero();
  imu_state.gyro_bias = Vector3d::Zero();
  imu_state.acc_bias = Vector3d::Zero();
  imu_state.orientation_null = Vector4d(0.0, 0.0, 0.0, 1.0);
  imu_state.position_null = Vector3d::Zero();
  imu_state.velocity_null = Vector3d::Zero();

  // Remove all existing camera states.
  state_server.cam_states.clear();

  // Reset the state covariance.
  double gyro_bias_cov, acc_bias_cov, velocity_cov;
  nh.param<double>("initial_covariance/velocity",
      velocity_cov, 0.25);
  nh.param<double>("initial_covariance/gyro_bias",
      gyro_bias_cov, 1e-4);
  nh.param<double>("initial_covariance/acc_bias",
      acc_bias_cov, 1e-2);

  double extrinsic_rotation_cov, extrinsic_translation_cov;
  nh.param<double>("initial_covariance/extrinsic_rotation_cov",
      extrinsic_rotation_cov, 3.0462e-4);
  nh.param<double>("initial_covariance/extrinsic_translation_cov",
      extrinsic_translation_cov, 1e-4);

  state_server.state_cov = MatrixXd::Zero(21, 21);
  for (int i = 3; i < 6; ++i)
    state_server.state_cov(i, i) = gyro_bias_cov;
  for (int i = 6; i < 9; ++i)
    state_server.state_cov(i, i) = velocity_cov;
  for (int i = 9; i < 12; ++i)
    state_server.state_cov(i, i) = acc_bias_cov;
  for (int i = 15; i < 18; ++i)
    state_server.state_cov(i, i) = extrinsic_rotation_cov;
  for (int i = 18; i < 21; ++i)
    state_server.state_cov(i, i) = extrinsic_translation_cov;

  // Clear all exsiting features in the map.
  map_server.clear();

  // Clear the IMU msg buffer.
  imu_msg_buffer.clear();

  // Reset the starting flags.
  is_gravity_set = false;
  is_first_img = true;

  // Restart the subscribers.
  imu_sub = nh.subscribe("imu", 100,
      &MsckfVio::imuCallback, this);
  feature_sub = nh.subscribe("features", 40,
      &MsckfVio::featureCallback, this);

  // TODO: When can the reset fail?
  res.success = true;
  ROS_WARN("Resetting msckf vio completed...");
  return true;
}
/*
 * 1.传播imu状态（状态预测）
 * 2.状态增广
 * 3.给特征点增加观测
 * 4.状态更新
 * 5.发布odometry
 * 6.检查系统是否需要复位
 */
void MsckfVio::featureCallback(
    const CameraMeasurementConstPtr& msg) {

  // Return if the gravity vector has not been set.
  //只有初始化重力后，才执行featureCallback
  if (!is_gravity_set) return;

  // Start the system if the first image is received.
  // The frame where the first image is received will be
  // the origin.
  if (is_first_img) {
    is_first_img = false;
    state_server.imu_state.time = msg->header.stamp.toSec();
  }

  static double max_processing_time = 0.0;
  static int critical_time_cntr = 0;
  double processing_start_time = ros::Time::now().toSec();

  // Propogate the IMU state.
  // that are received before the image msg.
  ros::Time start_time = ros::Time::now();
  batchImuProcessing(msg->header.stamp.toSec());
  double imu_processing_time = (
      ros::Time::now()-start_time).toSec();

  // Augment the state vector.
  start_time = ros::Time::now();
  stateAugmentation(msg->header.stamp.toSec());
  double state_augmentation_time = (
      ros::Time::now()-start_time).toSec();

  // Add new observations for existing features or new
  // features in the map server.
  start_time = ros::Time::now();
  addFeatureObservations(msg);
  double add_observations_time = (
      ros::Time::now()-start_time).toSec();

  // Perform measurement update if necessary.
  start_time = ros::Time::now();
  removeLostFeatures();
  double remove_lost_features_time = (
      ros::Time::now()-start_time).toSec();

  start_time = ros::Time::now();
  pruneCamStateBuffer();
  double prune_cam_states_time = (
      ros::Time::now()-start_time).toSec();

  // Publish the odometry.
  start_time = ros::Time::now();
  publish(msg->header.stamp);
  double publish_time = (
      ros::Time::now()-start_time).toSec();

  // Reset the system if necessary.
  onlineReset();

  double processing_end_time = ros::Time::now().toSec();
  double processing_time =
    processing_end_time - processing_start_time;
  if (processing_time > 1.0/frame_rate) {
    ++critical_time_cntr;
    ROS_INFO("\033[1;31mTotal processing time %f/%d...\033[0m",
        processing_time, critical_time_cntr);
    //printf("IMU processing time: %f/%f\n",
    //    imu_processing_time, imu_processing_time/processing_time);
    //printf("State augmentation time: %f/%f\n",
    //    state_augmentation_time, state_augmentation_time/processing_time);
    //printf("Add observations time: %f/%f\n",
    //    add_observations_time, add_observations_time/processing_time);
    printf("Remove lost features time: %f/%f\n",
        remove_lost_features_time, remove_lost_features_time/processing_time);
    printf("Remove camera states time: %f/%f\n",
        prune_cam_states_time, prune_cam_states_time/processing_time);
    //printf("Publish time: %f/%f\n",
    //    publish_time, publish_time/processing_time);
  }

  return;
}

void MsckfVio::mocapOdomCallback(
    const nav_msgs::OdometryConstPtr& msg) {
  static bool first_mocap_odom_msg = true;

  // If this is the first mocap odometry messsage, set
  // the initial frame.
  if (first_mocap_odom_msg) {
    Quaterniond orientation;
    Vector3d translation;
    tf::pointMsgToEigen(
        msg->pose.pose.position, translation);
    tf::quaternionMsgToEigen(
        msg->pose.pose.orientation, orientation);
    //tf::vectorMsgToEigen(
    //    msg->transform.translation, translation);
    //tf::quaternionMsgToEigen(
    //    msg->transform.rotation, orientation);
    mocap_initial_frame.linear() = orientation.toRotationMatrix();
    mocap_initial_frame.translation() = translation;
    first_mocap_odom_msg = false;
  }

  // Transform the ground truth.
  Quaterniond orientation;
  Vector3d translation;
  //tf::vectorMsgToEigen(
  //    msg->transform.translation, translation);
  //tf::quaternionMsgToEigen(
  //    msg->transform.rotation, orientation);
  tf::pointMsgToEigen(
      msg->pose.pose.position, translation);
  tf::quaternionMsgToEigen(
      msg->pose.pose.orientation, orientation);

  Eigen::Isometry3d T_b_v_gt;
  T_b_v_gt.linear() = orientation.toRotationMatrix();
  T_b_v_gt.translation() = translation;
  Eigen::Isometry3d T_b_w_gt = mocap_initial_frame.inverse() * T_b_v_gt;

  //Eigen::Vector3d body_velocity_gt;
  //tf::vectorMsgToEigen(msg->twist.twist.linear, body_velocity_gt);
  //body_velocity_gt = mocap_initial_frame.linear().transpose() *
  //  body_velocity_gt;

  // Ground truth tf.
  if (publish_tf) {
    tf::Transform T_b_w_gt_tf;
    tf::transformEigenToTF(T_b_w_gt, T_b_w_gt_tf);
    tf_pub.sendTransform(tf::StampedTransform(
          T_b_w_gt_tf, msg->header.stamp, fixed_frame_id, child_frame_id+"_mocap"));
  }

  // Ground truth odometry.
  nav_msgs::Odometry mocap_odom_msg;
  mocap_odom_msg.header.stamp = msg->header.stamp;
  mocap_odom_msg.header.frame_id = fixed_frame_id;
  mocap_odom_msg.child_frame_id = child_frame_id+"_mocap";

  tf::poseEigenToMsg(T_b_w_gt, mocap_odom_msg.pose.pose);
  //tf::vectorEigenToMsg(body_velocity_gt,
  //    mocap_odom_msg.twist.twist.linear);

  mocap_odom_pub.publish(mocap_odom_msg);
  return;
}
/*预测过程：
 * 1.状态量的预测（数值积分）
 * 2.协方差的传播
 * 3.删除用过的IMU信息
 */
void MsckfVio::batchImuProcessing(const double& time_bound) {
  // Counter how many IMU msgs in the buffer are used.
  int used_imu_msg_cntr = 0;

  for (const auto& imu_msg : imu_msg_buffer) {
    double imu_time = imu_msg.header.stamp.toSec();
    //如果imu时间戳小于图像时间戳
    if (imu_time < state_server.imu_state.time) {
      ++used_imu_msg_cntr;
      continue;
    }
    //如果imu时间戳大于图像时间戳
    if (imu_time > time_bound) break;

    //如果imu时间戳=图像时间戳
    // Convert the msgs.
    Vector3d m_gyro, m_acc;
    tf::vectorMsgToEigen(imu_msg.angular_velocity, m_gyro);
    tf::vectorMsgToEigen(imu_msg.linear_acceleration, m_acc);

    // Execute process model.
    processModel(imu_time, m_gyro, m_acc);
    ++used_imu_msg_cntr;
  }

  // Set the state ID for the new IMU state.
  state_server.imu_state.id = IMUState::next_id++;

  // Remove all used IMU msgs.
  imu_msg_buffer.erase(imu_msg_buffer.begin(),
      imu_msg_buffer.begin()+used_imu_msg_cntr);

  return;
}
/* 预测阶段，状态量估计和协方差传播,零空间更新
 * 输入：时间戳
 * 输入： m_gyro和m_acc的测量值
 */
void MsckfVio::processModel(const double& time,
    const Vector3d& m_gyro,
    const Vector3d& m_acc) {

  // Remove the bias from the measured gyro and acceleration
  //估计值w^ a^
  IMUState& imu_state = state_server.imu_state;
  Vector3d gyro = m_gyro - imu_state.gyro_bias;
  Vector3d acc = m_acc - imu_state.acc_bias;
  //IMU时间戳减去图像时间戳
  double dtime = time - imu_state.time;
//step1：协方差传播
  // Compute discrete transition and noise covariance matrix
  //计算F和G
  Matrix<double, 21, 21> F = Matrix<double, 21, 21>::Zero();
  Matrix<double, 21, 12> G = Matrix<double, 21, 12>::Zero();

  F.block<3, 3>(0, 0) = -skewSymmetric(gyro);
  F.block<3, 3>(0, 3) = -Matrix3d::Identity();
  F.block<3, 3>(6, 0) = -quaternionToRotation(
      imu_state.orientation).transpose()*skewSymmetric(acc);
  F.block<3, 3>(6, 9) = -quaternionToRotation(
      imu_state.orientation).transpose();
  F.block<3, 3>(12, 6) = Matrix3d::Identity();

  G.block<3, 3>(0, 0) = -Matrix3d::Identity();
  G.block<3, 3>(3, 3) = Matrix3d::Identity();
  G.block<3, 3>(6, 6) = -quaternionToRotation(
      imu_state.orientation).transpose();
  //和论文公式不一致，论文公式有错
  //博客推导正确  https://blog.csdn.net/liu2015302026/article/details/105339648
  G.block<3, 3>(9, 9) = Matrix3d::Identity();

  // Approximate matrix exponential to the 3rd order,
  // which can be considered to be accurate enough assuming
  // dtime is within 0.01s.
  //连续模型离散化
  //泰勒3阶展开，计算fai
  Matrix<double, 21, 21> Fdt = F * dtime;
  Matrix<double, 21, 21> Fdt_square = Fdt * Fdt;
  Matrix<double, 21, 21> Fdt_cube = Fdt_square * Fdt;
  Matrix<double, 21, 21> Phi = Matrix<double, 21, 21>::Identity() +
    Fdt + 0.5*Fdt_square + (1.0/6.0)*Fdt_cube;

  //预测阶段估计状态qvp
//step2：状态量估计
  predictNewState(dtime, gyro, acc);

  // Modify the transition matrix
  //todo 怎么理解？
  Matrix3d R_kk_1 = quaternionToRotation(imu_state.orientation_null);
  Phi.block<3, 3>(0, 0) =
    quaternionToRotation(imu_state.orientation) * R_kk_1.transpose();

  Vector3d u = R_kk_1 * IMUState::gravity;
  RowVector3d s = (u.transpose()*u).inverse() * u.transpose();

  Matrix3d A1 = Phi.block<3, 3>(6, 0);
  Vector3d w1 = skewSymmetric(
      imu_state.velocity_null-imu_state.velocity) * IMUState::gravity;
  Phi.block<3, 3>(6, 0) = A1 - (A1*u-w1)*s;

  Matrix3d A2 = Phi.block<3, 3>(12, 0);
  Vector3d w2 = skewSymmetric(
      dtime*imu_state.velocity_null+imu_state.position_null-
      imu_state.position) * IMUState::gravity;
  Phi.block<3, 3>(12, 0) = A2 - (A2*u-w2)*s;

  // Propogate the state covariance matrix.
  //传播协方差矩阵
  //Q_k
  Matrix<double, 21, 21> Q = Phi*G*state_server.continuous_noise_cov*
    G.transpose()*Phi.transpose()*dtime;
  //P_II
  state_server.state_cov.block<21, 21>(0, 0) =
    Phi*state_server.state_cov.block<21, 21>(0, 0)*Phi.transpose() + Q;
  //P_ic
  //P_ci
  if (state_server.cam_states.size() > 0) {
    state_server.state_cov.block(
        0, 21, 21, state_server.state_cov.cols()-21) =
      Phi * state_server.state_cov.block(
        0, 21, 21, state_server.state_cov.cols()-21);
    state_server.state_cov.block(
        21, 0, state_server.state_cov.rows()-21, 21) =
      state_server.state_cov.block(
        21, 0, state_server.state_cov.rows()-21, 21) * Phi.transpose();
  }
//强制使state_server.state_cov形成对角矩阵形式，方法，对角线两端元素相加取平均
  MatrixXd state_cov_fixed = (state_server.state_cov +
      state_server.state_cov.transpose()) / 2.0;
  state_server.state_cov = state_cov_fixed;

//step3:更新状态的零空间
  //todo，为什么这样赋值？
  imu_state.orientation_null = imu_state.orientation;
  imu_state.position_null = imu_state.position;
  imu_state.velocity_null = imu_state.velocity;

  // Update the state info
  state_server.imu_state.time = time;
  return;
}
/* IMU预测阶段估计状态
 * RK4估计新状态v，p
 * 0阶积分估计q
 *输入：时间间隔dt
 *输入：acc和gyro估计值（去除bias的）
 *输出：成员函数
 *     state_server.imu_state.orientation
 *     state_server.imu_state.velocity
 *     state_server.imu_state.position
 */
void MsckfVio::predictNewState(const double& dt,
    const Vector3d& gyro,
    const Vector3d& acc) {

  // TODO: Will performing the forward integration using
  //    the inverse of the quaternion give better accuracy?
  double gyro_norm = gyro.norm();
  //构建四元数乘法用到的Omega矩阵
  Matrix4d Omega = Matrix4d::Zero();
  Omega.block<3, 3>(0, 0) = -skewSymmetric(gyro);
  Omega.block<3, 1>(0, 3) = gyro;
  Omega.block<1, 3>(3, 0) = -gyro;

  Vector4d& q = state_server.imu_state.orientation;
  Vector3d& v = state_server.imu_state.velocity;
  Vector3d& p = state_server.imu_state.position;

  // Some pre-calculation
  //step1：0阶积分估计q  q_w_I
  //dq_dt对q 0阶积分，时间间隔取deta_t
  //dq_dt2对q 0阶积分,时间间隔取deta_t/2

  Vector4d dq_dt, dq_dt2;
  if (gyro_norm > 1e-5) {
    dq_dt = (cos(gyro_norm*dt*0.5)*Matrix4d::Identity() +
      1/gyro_norm*sin(gyro_norm*dt*0.5)*Omega) * q;
    dq_dt2 = (cos(gyro_norm*dt*0.25)*Matrix4d::Identity() +
      1/gyro_norm*sin(gyro_norm*dt*0.25)*Omega) * q;
  }
  else {
      //todo 与知乎公式不同？
    dq_dt = (Matrix4d::Identity()+0.5*dt*Omega) *
      cos(gyro_norm*dt*0.5) * q;
    dq_dt2 = (Matrix4d::Identity()+0.25*dt*Omega) *
      cos(gyro_norm*dt*0.25) * q;
  }
  Matrix3d dR_dt_transpose = quaternionToRotation(dq_dt).transpose();
  Matrix3d dR_dt2_transpose = quaternionToRotation(dq_dt2).transpose();
//step2：RK4估计p，v
  // k1 = f(tn, yn)
  //计算使用RK4近似v，p （注意旋转矩阵所属时刻）
  Vector3d k1_v_dot = quaternionToRotation(q).transpose()*acc +
    IMUState::gravity;
  Vector3d k1_p_dot = v;

  // k2 = f(tn+dt/2, yn+k1*dt/2)
  Vector3d k1_v = v + k1_v_dot*dt/2;
  Vector3d k2_v_dot = dR_dt2_transpose*acc +
    IMUState::gravity;
  Vector3d k2_p_dot = k1_v;

  // k3 = f(tn+dt/2, yn+k2*dt/2)
  Vector3d k2_v = v + k2_v_dot*dt/2;
  Vector3d k3_v_dot = dR_dt2_transpose*acc +
    IMUState::gravity;
  Vector3d k3_p_dot = k2_v;

  // k4 = f(tn+dt, yn+k3*dt)
  Vector3d k3_v = v + k3_v_dot*dt;
  Vector3d k4_v_dot = dR_dt_transpose*acc +
    IMUState::gravity;
  Vector3d k4_p_dot = k3_v;

  // yn+1 = yn + dt/6*(k1+2*k2+2*k3+k4)
  q = dq_dt;
  quaternionNormalize(q);
  v = v + dt/6*(k1_v_dot+2*k2_v_dot+2*k3_v_dot+k4_v_dot);
  p = p + dt/6*(k1_p_dot+2*k2_p_dot+2*k3_p_dot+k4_p_dot);

  return;
}
/*状态增广阶段
 * step1：相机状态增广，使用IMU状态的估计值和外参来估计cam状态
 * step2：协方差矩阵增广
 * step3：使协方差矩阵变为对称矩阵，对角线元素相加后取平均
 * 输入：cam时间戳
 */
void MsckfVio::stateAugmentation(const double& time) {

  const Matrix3d& R_i_c = state_server.imu_state.R_imu_cam0;
  const Vector3d& t_c_i = state_server.imu_state.t_cam0_imu;

  // Add a new camera state to the state server.
  Matrix3d R_w_i = quaternionToRotation(
      state_server.imu_state.orientation);
//step1：相机状态增广
//和原论文公式不一致，原论文有错
  Matrix3d R_w_c = R_i_c * R_w_i;
  Vector3d t_c_w = state_server.imu_state.position +
    R_w_i.transpose()*t_c_i;
//todo 相同时刻的相机id与imu id好像是相同的
  state_server.cam_states[state_server.imu_state.id] =
    CAMState(state_server.imu_state.id);
  CAMState& cam_state = state_server.cam_states[
    state_server.imu_state.id];

  cam_state.time = time;
  cam_state.orientation = rotationToQuaternion(R_w_c);
  cam_state.position = t_c_w;
//todo 为什么这样？
  cam_state.orientation_null = cam_state.orientation;
  cam_state.position_null = cam_state.position;

  // Update the covariance matrix of the state.
  // To simplify computation, the matrix J below is the nontrivial block
  // in Equation (16) in "A Multi-State Constraint Kalman Filter for Vision
  // -aided Inertial Navigation".
  //J
//step2：协方差矩阵增广
  Matrix<double, 6, 21> J = Matrix<double, 6, 21>::Zero();
  J.block<3, 3>(0, 0) = R_i_c;
  J.block<3, 3>(0, 15) = Matrix3d::Identity();
  //todo，这个和原论文公式不一致啊？
  J.block<3, 3>(3, 0) = skewSymmetric(R_w_i.transpose()*t_c_i);
  //J.block<3, 3>(3, 0) = -R_w_i.transpose()*skewSymmetric(t_c_i);
  J.block<3, 3>(3, 12) = Matrix3d::Identity();
  J.block<3, 3>(3, 18) = Matrix3d::Identity();

  // Resize the state covariance matrix.
  size_t old_rows = state_server.state_cov.rows();
  size_t old_cols = state_server.state_cov.cols();
  //在不改变原有元素的情况下改变矩阵维度
  state_server.state_cov.conservativeResize(old_rows+6, old_cols+6);

  // Rename some matrix blocks for convenience.
  const Matrix<double, 21, 21>& P11 =
    state_server.state_cov.block<21, 21>(0, 0);
  const MatrixXd& P12 =
    state_server.state_cov.block(0, 21, 21, old_cols-21);

  // 分块填充协方差矩阵
  state_server.state_cov.block(old_rows, 0, 6, old_cols) << J*P11, J*P12;
  state_server.state_cov.block(0, old_cols, old_rows, 6) =
    state_server.state_cov.block(old_rows, 0, 6, old_cols).transpose();
  state_server.state_cov.block<6, 6>(old_rows, old_cols) =
    J * P11 * J.transpose();

  // Fix the covariance to be symmetric
//step3：使协方差矩阵变为对称矩阵，对角线元素相加后取平均
  MatrixXd state_cov_fixed = (state_server.state_cov +
      state_server.state_cov.transpose()) / 2.0;
  state_server.state_cov = state_cov_fixed;

  return;
}
/*将topic传来的特征点添加map_server
 * 1.如果是新特征，就在map_server中创建新特征，给新特征添加观测
 * 2.如果是旧特征,则将当前帧加入到map_server该点的观测之中
 * 输入：ros传来的CameraMeasurement类型的消息
 * 输出：成员变量map_server
 */
void MsckfVio::addFeatureObservations(
    const CameraMeasurementConstPtr& msg) {

  StateIDType state_id = state_server.imu_state.id;
  int curr_feature_num = map_server.size();
  int tracked_feature_num = 0;

  // Add new observations for existing features or new
  // features in the map server.
  for (const auto& feature : msg->features) {
    if (map_server.find(feature.id) == map_server.end()) {
      // 如果是新特征，创建新特征，给新特征添加观测
      map_server[feature.id] = Feature(feature.id);
      map_server[feature.id].observations[state_id] =
        Vector4d(feature.u0, feature.v0,
            feature.u1, feature.v1);
    } else {
      // 如果是旧特征，给已有特征添加观测
      map_server[feature.id].observations[state_id] =
        Vector4d(feature.u0, feature.v0,
            feature.u1, feature.v1);
      ++tracked_feature_num;
    }
  }

  tracking_rate =
    static_cast<double>(tracked_feature_num) /
    static_cast<double>(curr_feature_num);

  return;
}
//论文公式（5）
//计算测量时期单个特帧点单次观测的雅可比，用于测量更新
//输入：观测到该特征点的第i个相机id cam_state_id
//输入：第j个特征点id feature_id
//输出：第i个观测残差对第i个cam0状态的雅可比H_x
//输出：第i个观测残差对world系下第j个特征点坐标的雅可比H_f
//输出：第i个观测残差r
void MsckfVio::measurementJacobian(
    const StateIDType& cam_state_id,
    const FeatureIDType& feature_id,
    Matrix<double, 4, 6>& H_x, Matrix<double, 4, 3>& H_f, Vector4d& r) {

  // Prepare all the required data.
  const CAMState& cam_state = state_server.cam_states[cam_state_id];
  const Feature& feature = map_server[feature_id];

  // Cam0 pose.
  Matrix3d R_w_c0 = quaternionToRotation(cam_state.orientation);
  const Vector3d& t_c0_w = cam_state.position;

  // Cam1 pose.
  Matrix3d R_c0_c1 = CAMState::T_cam0_cam1.linear();
  Matrix3d R_w_c1 = CAMState::T_cam0_cam1.linear() * R_w_c0;
  Vector3d t_c1_w = t_c0_w - R_w_c1.transpose()*CAMState::T_cam0_cam1.translation();

  // 3d feature position in the world frame.
  // And its observation with the stereo cameras.
  const Vector3d& p_w = feature.position;
  const Vector4d& z = feature.observations.find(cam_state_id)->second;

  // Convert the feature position from the world frame to
  // the cam0 and cam1 frame.
  Vector3d p_c0 = R_w_c0 * (p_w-t_c0_w);
  Vector3d p_c1 = R_w_c1 * (p_w-t_c1_w);

  // Compute the Jacobians.
//观测残差对cam0系下的特征点位置的雅可比
  Matrix<double, 4, 3> dz_dpc0 = Matrix<double, 4, 3>::Zero();
  dz_dpc0(0, 0) = 1 / p_c0(2);
  dz_dpc0(1, 1) = 1 / p_c0(2);
  dz_dpc0(0, 2) = -p_c0(0) / (p_c0(2)*p_c0(2));
  dz_dpc0(1, 2) = -p_c0(1) / (p_c0(2)*p_c0(2));
//观测残差对cam1系下的特征点位置的雅可比 
  Matrix<double, 4, 3> dz_dpc1 = Matrix<double, 4, 3>::Zero();
  dz_dpc1(2, 0) = 1 / p_c1(2);
  dz_dpc1(3, 1) = 1 / p_c1(2);
  dz_dpc1(2, 2) = -p_c1(0) / (p_c1(2)*p_c1(2));
  dz_dpc1(3, 2) = -p_c1(1) / (p_c1(2)*p_c1(2));
//cam0系下的特征点位置对cam0状态的雅可比
  Matrix<double, 3, 6> dpc0_dxc = Matrix<double, 3, 6>::Zero();
  dpc0_dxc.leftCols(3) = skewSymmetric(p_c0);
  dpc0_dxc.rightCols(3) = -R_w_c0;
//cam1系下的特征点位置对cam0状态的雅可比
  Matrix<double, 3, 6> dpc1_dxc = Matrix<double, 3, 6>::Zero();
  dpc1_dxc.leftCols(3) = R_c0_c1 * skewSymmetric(p_c0);
  dpc1_dxc.rightCols(3) = -R_w_c1;
//cam0系下的特征点位置对world系下的特征点位置的雅可比
  Matrix3d dpc0_dpg = R_w_c0;
//cam1系下的特征点位置对world系下的特征点位置的雅可比
  Matrix3d dpc1_dpg = R_w_c1;
//原论文公式（7）
//第i个观测残差对相应cam0状态的雅可比
  H_x = dz_dpc0*dpc0_dxc + dz_dpc1*dpc1_dxc;
//第i个观测残差对world系下的特征点位置的雅可比
  H_f = dz_dpc0*dpc0_dpg + dz_dpc1*dpc1_dpg;
//todo，什么意思？
  // Modifty the measurement Jacobian to ensure
  // observability constrain.
  Matrix<double, 4, 6> A = H_x;
  Matrix<double, 6, 1> u = Matrix<double, 6, 1>::Zero();
  u.block<3, 1>(0, 0) = quaternionToRotation(
      cam_state.orientation_null) * IMUState::gravity;
  u.block<3, 1>(3, 0) = skewSymmetric(
      p_w-cam_state.position_null) * IMUState::gravity;
  H_x = A - A*u*(u.transpose()*u).inverse()*u.transpose();
  H_f = -H_x.block<4, 3>(0, 3);

  // Compute the residual.
  //计算归一化相机坐标残差
  r = z - Vector4d(p_c0(0)/p_c0(2), p_c0(1)/p_c0(2),
      p_c1(0)/p_c1(2), p_c1(1)/p_c1(2));

  return;
}

/*计算测量阶段的雅可比，论文公式（6）
 * 输入：特征id
 *输入：所有相机id
 *输出：H_xo_j
 *输出：r_o_j
 */
void MsckfVio::featureJacobian(
    const FeatureIDType& feature_id,
    const std::vector<StateIDType>& cam_state_ids,
    MatrixXd& H_x, VectorXd& r) {

  const auto& feature = map_server[feature_id];

  // Check how many camera states in the provided camera
  // id camera has actually seen this feature.
  vector<StateIDType> valid_cam_state_ids(0);
  for (const auto& cam_id : cam_state_ids) {
    if (feature.observations.find(cam_id) ==
        feature.observations.end()) continue;

    valid_cam_state_ids.push_back(cam_id);
  }

  int jacobian_row_size = 0;
  jacobian_row_size = 4 * valid_cam_state_ids.size();

  MatrixXd H_xj = MatrixXd::Zero(jacobian_row_size,
      21+state_server.cam_states.size()*6);
  MatrixXd H_fj = MatrixXd::Zero(jacobian_row_size, 3);
  VectorXd r_j = VectorXd::Zero(jacobian_row_size);
  int stack_cntr = 0;

  for (const auto& cam_id : valid_cam_state_ids) {

    Matrix<double, 4, 6> H_xi = Matrix<double, 4, 6>::Zero();
    Matrix<double, 4, 3> H_fi = Matrix<double, 4, 3>::Zero();
    Vector4d r_i = Vector4d::Zero();
    measurementJacobian(cam_id, feature.id, H_xi, H_fi, r_i);

    auto cam_state_iter = state_server.cam_states.find(cam_id);
    int cam_state_cntr = std::distance(
        state_server.cam_states.begin(), cam_state_iter);

    // Stack the Jacobians.
    //堆叠雅可比
    //H_xj为第i个特征点的所有观测残差对所有状态变量的雅可比
    H_xj.block<4, 6>(stack_cntr, 21+6*cam_state_cntr) = H_xi;
    //H_fj 为第i个特征点的所有观测残差对world系下特征点坐标的雅可比
    H_fj.block<4, 3>(stack_cntr, 0) = H_fi;
    r_j.segment<4>(stack_cntr) = r_i;
    stack_cntr += 4;
  }

  // Project the residual and Jacobians onto the nullspace
  // of H_fj.
  //todo，为什么零空间用svd这样求？
  JacobiSVD<MatrixXd> svd_helper(H_fj, ComputeFullU | ComputeThinV);
  MatrixXd A = svd_helper.matrixU().rightCols(
      jacobian_row_size - 3);

  H_x = A.transpose() * H_xj;
  r = A.transpose() * r_j;

  return;
}
//最开始的msckf论文公式（22）
/*EKF更新阶段
 *step1.更新状态量（IMU+cam）
 *step2.更新协方差矩阵
 *输入：总的雅可比H_x
 *输入：总的测量残差r_o
 *输出：状态量和协方差（成员变量）
 */
void MsckfVio::measurementUpdate(
    const MatrixXd& H, const VectorXd& r) {

  if (H.rows() == 0 || r.rows() == 0) return;

  // Decompose the final Jacobian matrix to reduce computational
  // complexity as in Equation (28), (29).
  MatrixXd H_thin;
  VectorXd r_thin;

  if (H.rows() > H.cols()) {
    // Convert H to a sparse matrix.
    SparseMatrix<double> H_sparse = H.sparseView();

    // Perform QR decompostion on H_sparse.
    //QR分解
    SPQR<SparseMatrix<double> > spqr_helper;
    spqr_helper.setSPQROrdering(SPQR_ORDERING_NATURAL);
    spqr_helper.compute(H_sparse);

    MatrixXd H_temp;
    VectorXd r_temp;
    (spqr_helper.matrixQ().transpose() * H).evalTo(H_temp);
    (spqr_helper.matrixQ().transpose() * r).evalTo(r_temp);
//T_H
    H_thin = H_temp.topRows(21+state_server.cam_states.size()*6);
//r_n
    r_thin = r_temp.head(21+state_server.cam_states.size()*6);

    //HouseholderQR<MatrixXd> qr_helper(H);
    //MatrixXd Q = qr_helper.householderQ();
    //MatrixXd Q1 = Q.leftCols(21+state_server.cam_states.size()*6);

    //H_thin = Q1.transpose() * H;
    //r_thin = Q1.transpose() * r;
  } else {
    H_thin = H;
    r_thin = r;
  }

  // Compute the Kalman gain.
  const MatrixXd& P = state_server.state_cov;
  MatrixXd S = H_thin*P*H_thin.transpose() +
      Feature::observation_noise*MatrixXd::Identity(
        H_thin.rows(), H_thin.rows());
  //MatrixXd K_transpose = S.fullPivHouseholderQr().solve(H_thin*P);
  //为了计算速度快，LDLT法求解S*K_transpose=H_thin*P
  //上式可由ekf中K的公式，推到所得
  MatrixXd K_transpose = S.ldlt().solve(H_thin*P);
  MatrixXd K = K_transpose.transpose();

  // Compute the error of the state.
  VectorXd delta_x = K * r_thin;

  // Update the IMU state.
  const VectorXd& delta_x_imu = delta_x.head<21>();
//当速度和平移增量很大时，输出相关信息
  if (//delta_x_imu.segment<3>(0).norm() > 0.15 ||
      //delta_x_imu.segment<3>(3).norm() > 0.15 ||
      delta_x_imu.segment<3>(6).norm() > 0.5 ||
      //delta_x_imu.segment<3>(9).norm() > 0.5 ||
      delta_x_imu.segment<3>(12).norm() > 1.0) {
    printf("delta velocity: %f\n", delta_x_imu.segment<3>(6).norm());
    printf("delta position: %f\n", delta_x_imu.segment<3>(12).norm());
    ROS_WARN("Update change is too large.");
    //return;
  }
//更新四元数delta_q*q
  const Vector4d dq_imu =
    smallAngleQuaternion(delta_x_imu.head<3>());
  state_server.imu_state.orientation = quaternionMultiplication(
      dq_imu, state_server.imu_state.orientation);
  state_server.imu_state.gyro_bias += delta_x_imu.segment<3>(3);
  state_server.imu_state.velocity += delta_x_imu.segment<3>(6);
  state_server.imu_state.acc_bias += delta_x_imu.segment<3>(9);
  state_server.imu_state.position += delta_x_imu.segment<3>(12);
//更新旋转矩阵delta_R*R
  const Vector4d dq_extrinsic =
    smallAngleQuaternion(delta_x_imu.segment<3>(15));
  state_server.imu_state.R_imu_cam0 = quaternionToRotation(
      dq_extrinsic) * state_server.imu_state.R_imu_cam0;
  state_server.imu_state.t_cam0_imu += delta_x_imu.segment<3>(18);

  // Update the camera states.
  auto cam_state_iter = state_server.cam_states.begin();
  for (int i = 0; i < state_server.cam_states.size();
      ++i, ++cam_state_iter) {
    const VectorXd& delta_x_cam = delta_x.segment<6>(21+i*6);
//更新四元数
    const Vector4d dq_cam = smallAngleQuaternion(delta_x_cam.head<3>());
    cam_state_iter->second.orientation = quaternionMultiplication(
        dq_cam, cam_state_iter->second.orientation);
    cam_state_iter->second.position += delta_x_cam.tail<3>();
  }

  //更新协方差矩阵
  MatrixXd I_KH = MatrixXd::Identity(K.rows(), H_thin.cols()) - K*H_thin;
  //state_server.state_cov = I_KH*state_server.state_cov*I_KH.transpose() +
  //  K*K.transpose()*Feature::observation_noise;
  state_server.state_cov = I_KH*state_server.state_cov;

  // Fix the covariance to be symmetric
  //保持对称
  MatrixXd state_cov_fixed = (state_server.state_cov +
      state_server.state_cov.transpose()) / 2.0;
  state_server.state_cov = state_cov_fixed;

  return;
}

bool MsckfVio::gatingTest(
    const MatrixXd& H, const VectorXd& r, const int& dof) {

  MatrixXd P1 = H * state_server.state_cov * H.transpose();
  MatrixXd P2 = Feature::observation_noise *
    MatrixXd::Identity(H.rows(), H.rows());
  double gamma = r.transpose() * (P1+P2).ldlt().solve(r);

  //cout << dof << " " << gamma << " " <<
  //  chi_squared_test_table[dof] << " ";

  if (gamma < chi_squared_test_table[dof]) {
    //cout << "passed" << endl;
    return true;
  } else {
    //cout << "failed" << endl;
    return false;
  }
}
//测量更新阶段
//step1：从跟丢的特征中筛选可以三角化，并进行三角化
/*筛选条件：step1.1：该特征的被观测次数>=3次
 *        step1.2：如果点没有被初始化，
 *                 1.该特征第一个和最后一个观测帧之间的运动足够大（视差大）
 *                 2.三角化后所有点都在相机前面
 *（已经初始化且满足step1.1的点可用于测量更新）
 */
//step2：进行EKF测量更新
//step3：删除所有跟丢的特征
void MsckfVio::removeLostFeatures() {

  // Remove the features that lost track.
  // BTW, find the size the final Jacobian matrix and residual vector.
  int jacobian_row_size = 0;
  vector<FeatureIDType> invalid_feature_ids(0);
  vector<FeatureIDType> processed_feature_ids(0);
//step1：从跟丢的特征中筛选可以三角化，并进行三角化
/*筛选条件：step1.1：该特征的被观测次数>=3次
 *        step1.2：该特征第一个和最后一个观测帧之间的运动足够大（视差大）
 *        step1.3：三角化后所有点都在相机前面
 */

  for (auto iter = map_server.begin();
      iter != map_server.end(); ++iter) {
    // Rename the feature to be checked.
    auto& feature = iter->second;

    // Pass the features that are still being tracked.
    //如果该特征没有被当前帧观测到，且该特征的被观测次数只有3次  ，则判定为invalid_feature
    if (feature.observations.find(state_server.imu_state.id) !=
        feature.observations.end()) continue;
    if (feature.observations.size() < 3) {
      invalid_feature_ids.push_back(feature.id);
      continue;
    }
    //如果该特征没有被当前帧观测到，且该特征的被观测次数大于等于3次
    // Check if the feature can be initialized if it
    // has not been.
    if (!feature.is_initialized) {
        //如果没有足够大的运动，判定该特征为invalid_feature
      if (!feature.checkMotion(state_server.cam_states)) {
        invalid_feature_ids.push_back(feature.id);
        continue;
      } else {
         //如果不满足三角化条件，则判断为invalid_feature
        if(!feature.initializePosition(state_server.cam_states)) {
          invalid_feature_ids.push_back(feature.id);
          continue;
        }
      }
    }

    jacobian_row_size += 4*feature.observations.size() - 3;
    //三角化后的特征
    processed_feature_ids.push_back(feature.id);
  }

  //cout << "invalid/processed feature #: " <<
  //  invalid_feature_ids.size() << "/" <<
  //  processed_feature_ids.size() << endl;
  //cout << "jacobian row #: " << jacobian_row_size << endl;

  // Remove the features that do not have enough measurements.
  for (const auto& feature_id : invalid_feature_ids)
    map_server.erase(feature_id);

  // Return if there is no lost feature to be processed.
  if (processed_feature_ids.size() == 0) return;
//step2：进行EKF测量更新
  MatrixXd H_x = MatrixXd::Zero(jacobian_row_size,
      21+6*state_server.cam_states.size());
  VectorXd r = VectorXd::Zero(jacobian_row_size);
  int stack_cntr = 0;

  // Process the features which lose track.
  for (const auto& feature_id : processed_feature_ids) {
    auto& feature = map_server[feature_id];

    vector<StateIDType> cam_state_ids(0);
    for (const auto& measurement : feature.observations)
      cam_state_ids.push_back(measurement.first);

    MatrixXd H_xj;
    VectorXd r_j;
    featureJacobian(feature.id, cam_state_ids, H_xj, r_j);
//todo，这段是干什么？
//将所有特征点的所有观测都堆叠在一起
    if (gatingTest(H_xj, r_j, cam_state_ids.size()-1)) {
      H_x.block(stack_cntr, 0, H_xj.rows(), H_xj.cols()) = H_xj;
      r.segment(stack_cntr, r_j.rows()) = r_j;
      stack_cntr += H_xj.rows();
    }

    // Put an upper bound on the row size of measurement Jacobian,
    // which helps guarantee the executation time.
    if (stack_cntr > 1500) break;
  }

  H_x.conservativeResize(stack_cntr, H_x.cols());
  r.conservativeResize(stack_cntr);

  // Perform the measurement update step.
  //EKF测量更新
  measurementUpdate(H_x, r);
//step3：删除所有跟丢的特征
  // Remove all processed features from the map.
  for (const auto& feature_id : processed_feature_ids)
    map_server.erase(feature_id);

  return;
}
//当cam state数达到阈值时，进行cam状态剔除,要剔除两帧
/*剔除策略：根据平移，旋转角度，特征跟踪率，判断次次新帧和次新帧距次次次新帧的运动大小
 *        如果运动小于阈值，就剔除对应的帧
 *        否则，剔除最老的帧
 *输出：相机状态id rm_cam_state_ids
 */
void MsckfVio::findRedundantCamStates(
    vector<StateIDType>& rm_cam_state_ids) {

  // Move the iterator to the key position.
  auto key_cam_state_iter = state_server.cam_states.end();
  for (int i = 0; i < 4; ++i)
      //指向次次次新帧
    --key_cam_state_iter;
  auto cam_state_iter = key_cam_state_iter;
  //指向倒数第三帧（次次新帧）
  ++cam_state_iter;
  auto first_cam_state_iter = state_server.cam_states.begin();

  // Pose of the key camera state.
  const Vector3d key_position =
    key_cam_state_iter->second.position;
  const Matrix3d key_rotation = quaternionToRotation(
      key_cam_state_iter->second.orientation);

  // Mark the camera states to be removed based on the
  // motion between states.
  for (int i = 0; i < 2; ++i) {
    const Vector3d position =
      cam_state_iter->second.position;
    const Matrix3d rotation = quaternionToRotation(
        cam_state_iter->second.orientation);
//两帧之间的距离
    double distance = (position-key_position).norm();
//两帧之间的旋转角度
    double angle = AngleAxisd(
        rotation*key_rotation.transpose()).angle();
//跟踪率越大，说明相机距前一帧的移动越小
//根据平移，旋转角度，特征跟踪率，判断次次新帧和次新帧距次次次新帧的运动
//如果运动小于阈值，就剔除对应的帧
    if (angle < rotation_threshold &&
        distance < translation_threshold &&
        tracking_rate > tracking_rate_threshold) {
      rm_cam_state_ids.push_back(cam_state_iter->first);
      ++cam_state_iter;
    } else {
//否则，剔除最老的帧
      rm_cam_state_ids.push_back(first_cam_state_iter->first);
      ++first_cam_state_iter;
    }
  }

  // Sort the elements in the output vector.
  //根据cam状态id排序（升序）
  sort(rm_cam_state_ids.begin(), rm_cam_state_ids.end());

  return;
}

//step1：找到应该剔除的cam状态
/*step2：遍历被待删帧观测到的所有特征点，
 *       如果该特征没初始化且不能初始化（三角化），则删除该特征点中待删相机的观测
 *       如果该特征没初始化且能初始化（三角化），则进行三角化，然后进行EKF测量更新
 *       最后删除该特征点中待删相机的观测，删除协方差矩阵的对应块，及对应的cam状态
 */
void MsckfVio::pruneCamStateBuffer() {

  if (state_server.cam_states.size() < max_cam_state_size)
    return;
//如果状态数量达到max_cam_state_size
  // Find two camera states to be removed.
  vector<StateIDType> rm_cam_state_ids(0);
//step1：找到应该剔除的cam状态
  findRedundantCamStates(rm_cam_state_ids);
/*step2：遍历被待删帧观测到的所有特征点，
 *       如果该特征不能初始化（三角化），则删除该特征点中待删相机的观测
 *       如果该特征能初始化（三角化），则进行三角化，然后进行EKF测量更新
 *       最后删除该特征点中待删相机的观测，删除协方差矩阵的对应块，及对应的cam状态
 */
  // Find the size of the Jacobian matrix.
  int jacobian_row_size = 0;
  for (auto& item : map_server) {
    auto& feature = item.second;
    // Check how many camera states to be removed are associated
    // with this feature.
    vector<StateIDType> involved_cam_state_ids(0);
    for (const auto& cam_id : rm_cam_state_ids) {
      if (feature.observations.find(cam_id) !=
          feature.observations.end())
        involved_cam_state_ids.push_back(cam_id);
    }

    if (involved_cam_state_ids.size() == 0) continue;
    if (involved_cam_state_ids.size() == 1) {
      feature.observations.erase(involved_cam_state_ids[0]);
      continue;
    }
    //如果该特征还没有被三角化（初始化）
    if (!feature.is_initialized) {
      // Check if the feature can be initialize.
      //如果不能初始化，删除特征点对应的相机观测
      if (!feature.checkMotion(state_server.cam_states)) {
        // If the feature cannot be initialized, just remove
        // the observations associated with the camera states
        // to be removed.
        for (const auto& cam_id : involved_cam_state_ids)
          feature.observations.erase(cam_id);
        continue;
      } else {
       //如果不能三角化则删除观测，否则进行三角化

        if(!feature.initializePosition(state_server.cam_states)) {
          for (const auto& cam_id : involved_cam_state_ids)
            feature.observations.erase(cam_id);
          continue;
        }
      }
    }

    jacobian_row_size += 4*involved_cam_state_ids.size() - 3;
  }
//能进行三角化的点会进行一次EKF更新，之后，删除对应观测
  //cout << "jacobian row #: " << jacobian_row_size << endl;

  // Compute the Jacobian and residual.
  MatrixXd H_x = MatrixXd::Zero(jacobian_row_size,
      21+6*state_server.cam_states.size());
  VectorXd r = VectorXd::Zero(jacobian_row_size);
  int stack_cntr = 0;

  for (auto& item : map_server) {
    auto& feature = item.second;
    // Check how many camera states to be removed are associated
    // with this feature.
    vector<StateIDType> involved_cam_state_ids(0);
    for (const auto& cam_id : rm_cam_state_ids) {
      if (feature.observations.find(cam_id) !=
          feature.observations.end())
        involved_cam_state_ids.push_back(cam_id);
    }

    if (involved_cam_state_ids.size() == 0) continue;

    MatrixXd H_xj;
    VectorXd r_j;
//计算单个特征观测的雅可比
    featureJacobian(feature.id, involved_cam_state_ids, H_xj, r_j);
//所有特征的所有观测叠加的雅可比
    if (gatingTest(H_xj, r_j, involved_cam_state_ids.size())) {
      H_x.block(stack_cntr, 0, H_xj.rows(), H_xj.cols()) = H_xj;
      r.segment(stack_cntr, r_j.rows()) = r_j;
      stack_cntr += H_xj.rows();
    }
//删除相应观测
    for (const auto& cam_id : involved_cam_state_ids)
      feature.observations.erase(cam_id);
  }

  H_x.conservativeResize(stack_cntr, H_x.cols());
  r.conservativeResize(stack_cntr);
//测量更新
  // Perform measurement update.
  measurementUpdate(H_x, r);

  for (const auto& cam_id : rm_cam_state_ids) {
    int cam_sequence = std::distance(state_server.cam_states.begin(),
        state_server.cam_states.find(cam_id));
    int cam_state_start = 21 + 6*cam_sequence;
    int cam_state_end = cam_state_start + 6;

    // Remove the corresponding rows and columns in the state
    // covariance matrix.
//删除协方差矩阵的对应块
    if (cam_state_end < state_server.state_cov.rows()) {
      state_server.state_cov.block(cam_state_start, 0,
          state_server.state_cov.rows()-cam_state_end,
          state_server.state_cov.cols()) =
        state_server.state_cov.block(cam_state_end, 0,
            state_server.state_cov.rows()-cam_state_end,
            state_server.state_cov.cols());

      state_server.state_cov.block(0, cam_state_start,
          state_server.state_cov.rows(),
          state_server.state_cov.cols()-cam_state_end) =
        state_server.state_cov.block(0, cam_state_end,
            state_server.state_cov.rows(),
            state_server.state_cov.cols()-cam_state_end);

      state_server.state_cov.conservativeResize(
          state_server.state_cov.rows()-6, state_server.state_cov.cols()-6);
    } else {
//如果待删的状态是最后一个
      state_server.state_cov.conservativeResize(
          state_server.state_cov.rows()-6, state_server.state_cov.cols()-6);
    }
//删除对应的cam状态
    // Remove this camera state in the state vector.
    state_server.cam_states.erase(cam_id);
  }

  return;
}

void MsckfVio::onlineReset() {

  // Never perform online reset if position std threshold
  // is non-positive.
  if (position_std_threshold <= 0) return;
  static long long int online_reset_counter = 0;

  // Check the uncertainty of positions to determine if
  // the system can be reset.
  double position_x_std = std::sqrt(state_server.state_cov(12, 12));
  double position_y_std = std::sqrt(state_server.state_cov(13, 13));
  double position_z_std = std::sqrt(state_server.state_cov(14, 14));

  if (position_x_std < position_std_threshold &&
      position_y_std < position_std_threshold &&
      position_z_std < position_std_threshold) return;

  ROS_WARN("Start %lld online reset procedure...",
      ++online_reset_counter);
  ROS_INFO("Stardard deviation in xyz: %f, %f, %f",
      position_x_std, position_y_std, position_z_std);

  // Remove all existing camera states.
  state_server.cam_states.clear();

  // Clear all exsiting features in the map.
  map_server.clear();

  // Reset the state covariance.
  double gyro_bias_cov, acc_bias_cov, velocity_cov;
  nh.param<double>("initial_covariance/velocity",
      velocity_cov, 0.25);
  nh.param<double>("initial_covariance/gyro_bias",
      gyro_bias_cov, 1e-4);
  nh.param<double>("initial_covariance/acc_bias",
      acc_bias_cov, 1e-2);

  double extrinsic_rotation_cov, extrinsic_translation_cov;
  nh.param<double>("initial_covariance/extrinsic_rotation_cov",
      extrinsic_rotation_cov, 3.0462e-4);
  nh.param<double>("initial_covariance/extrinsic_translation_cov",
      extrinsic_translation_cov, 1e-4);

  state_server.state_cov = MatrixXd::Zero(21, 21);
  for (int i = 3; i < 6; ++i)
    state_server.state_cov(i, i) = gyro_bias_cov;
  for (int i = 6; i < 9; ++i)
    state_server.state_cov(i, i) = velocity_cov;
  for (int i = 9; i < 12; ++i)
    state_server.state_cov(i, i) = acc_bias_cov;
  for (int i = 15; i < 18; ++i)
    state_server.state_cov(i, i) = extrinsic_rotation_cov;
  for (int i = 18; i < 21; ++i)
    state_server.state_cov(i, i) = extrinsic_translation_cov;

  ROS_WARN("%lld online reset complete...", online_reset_counter);
  return;
}

void MsckfVio::publish(const ros::Time& time) {

  // Convert the IMU frame to the body frame.
  const IMUState& imu_state = state_server.imu_state;
  Eigen::Isometry3d T_i_w = Eigen::Isometry3d::Identity();
  T_i_w.linear() = quaternionToRotation(
      imu_state.orientation).transpose();
  T_i_w.translation() = imu_state.position;
//todo 为什么这样算？
  Eigen::Isometry3d T_b_w = IMUState::T_imu_body * T_i_w *
    IMUState::T_imu_body.inverse();
  Eigen::Vector3d body_velocity =
    IMUState::T_imu_body.linear() * imu_state.velocity;
  //广播坐标系
  // Publish tf
  if (publish_tf) {
    tf::Transform T_b_w_tf;
    tf::transformEigenToTF(T_b_w, T_b_w_tf);
    tf_pub.sendTransform(tf::StampedTransform(
          T_b_w_tf, time, fixed_frame_id, child_frame_id));
  }

  // Publish the odometry
  nav_msgs::Odometry odom_msg;
  odom_msg.header.stamp = time;
  odom_msg.header.frame_id = fixed_frame_id;
  odom_msg.child_frame_id = child_frame_id;

  tf::poseEigenToMsg(T_b_w, odom_msg.pose.pose);
  tf::vectorEigenToMsg(body_velocity, odom_msg.twist.twist.linear);

  // Convert the covariance.
  Matrix3d P_oo = state_server.state_cov.block<3, 3>(0, 0);
  Matrix3d P_op = state_server.state_cov.block<3, 3>(0, 12);
  Matrix3d P_po = state_server.state_cov.block<3, 3>(12, 0);
  Matrix3d P_pp = state_server.state_cov.block<3, 3>(12, 12);
  Matrix<double, 6, 6> P_imu_pose = Matrix<double, 6, 6>::Zero();
  P_imu_pose << P_pp, P_po, P_op, P_oo;

  Matrix<double, 6, 6> H_pose = Matrix<double, 6, 6>::Zero();
  H_pose.block<3, 3>(0, 0) = IMUState::T_imu_body.linear();
  H_pose.block<3, 3>(3, 3) = IMUState::T_imu_body.linear();
  Matrix<double, 6, 6> P_body_pose = H_pose *
    P_imu_pose * H_pose.transpose();

  for (int i = 0; i < 6; ++i)
    for (int j = 0; j < 6; ++j)
      odom_msg.pose.covariance[6*i+j] = P_body_pose(i, j);

  // Construct the covariance for the velocity.
  Matrix3d P_imu_vel = state_server.state_cov.block<3, 3>(6, 6);
  Matrix3d H_vel = IMUState::T_imu_body.linear();
  Matrix3d P_body_vel = H_vel * P_imu_vel * H_vel.transpose();
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
      odom_msg.twist.covariance[i*6+j] = P_body_vel(i, j);

  odom_pub.publish(odom_msg);

  // Publish the 3D positions of the features that
  // has been initialized.
  pcl::PointCloud<pcl::PointXYZ>::Ptr feature_msg_ptr(
      new pcl::PointCloud<pcl::PointXYZ>());
  feature_msg_ptr->header.frame_id = fixed_frame_id;
  feature_msg_ptr->height = 1;
  for (const auto& item : map_server) {
    const auto& feature = item.second;
    if (feature.is_initialized) {
      Vector3d feature_position =
        IMUState::T_imu_body.linear() * feature.position;
      feature_msg_ptr->points.push_back(pcl::PointXYZ(
            feature_position(0), feature_position(1), feature_position(2)));
    }
  }
  feature_msg_ptr->width = feature_msg_ptr->points.size();

  feature_pub.publish(feature_msg_ptr);

  return;
}

} // namespace msckf_vio

