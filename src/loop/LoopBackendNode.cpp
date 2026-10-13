#include "loop/LoopBackendNode.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include <Eigen/Geometry>
#include <gtsam/linear/LossFunctions.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/msg/marker.hpp>

namespace super_lio_slam {
namespace {

gtsam::Vector6 poseSigmas(double rotation, double translation) {
  gtsam::Vector6 sigma;
  sigma << rotation, rotation, rotation, translation, translation, translation;
  return sigma;
}

double degrees(double radians) { return radians * 180.0 / M_PI; }

}  // namespace

LoopBackendNode::LoopBackendNode() : rclcpp::Node("loop_backend_node") {
  // STD uses its own contiguous index for descriptors and plane clouds. It is
  // deliberately independent of the incoming ID so reconnects are harmless.
  std_manager_.current_frame_id_ = 0;
  read_parameters(*this, std_manager_.config_setting_);

  keyframe_cloud_voxel_size_ = declare_parameter<double>("keyframe_cloud_voxel_size", 0.25);
  keyframe_cloud_max_points_ = declare_parameter<int>("keyframe_cloud_max_points", 30000);
  icp_voxel_size_ = declare_parameter<double>("icp_voxel_size", 0.5);
  icp_max_correspondence_distance_ =
      declare_parameter<double>("icp_max_correspondence_distance", 2.0);
  icp_max_iterations_ = declare_parameter<int>("icp_max_iterations", 50);
  icp_fitness_threshold_ = declare_parameter<double>("icp_fitness_threshold", 0.8);
  icp_min_overlap_ratio_ = declare_parameter<double>("icp_min_overlap_ratio", 0.35);
  icp_max_translation_correction_ =
      declare_parameter<double>("icp_max_translation_correction", 5.0);
  icp_max_rotation_correction_deg_ =
      declare_parameter<double>("icp_max_rotation_correction_deg", 20.0);
  icp_submap_half_width_ = declare_parameter<int>("icp_submap_half_width", 5);
  icp_min_points_ = declare_parameter<int>("icp_min_points", 100);
  icp_max_source_points_ = declare_parameter<int>("icp_max_source_points", 10000);
  icp_max_target_points_ = declare_parameter<int>("icp_max_target_points", 50000);
  loop_min_interval_ = declare_parameter<int>("loop_min_interval", 20);
  map_publish_stride_ = declare_parameter<int>("map_publish_stride", 30);
  map_max_keyframes_ = declare_parameter<int>("map_max_keyframes", 2000);
  map_max_points_per_keyframe_ =
      declare_parameter<int>("map_max_points_per_keyframe", 1000);
  map_voxel_size_ = declare_parameter<double>("map_voxel_size", 0.5);
  path_max_poses_ = declare_parameter<int>("path_max_poses", 10000);
  std_visualization_max_triangles_ =
      declare_parameter<int>("std_visualization_max_triangles", 100);
  tf_publish_rate_hz_ = declare_parameter<double>("tf_publish_rate_hz", 20.0);

  const double prior_trans = declare_parameter<double>("graph_prior_translation_sigma", 0.001);
  const double prior_rot = declare_parameter<double>("graph_prior_rotation_sigma", 0.001);
  const double odom_trans = declare_parameter<double>("graph_odom_translation_sigma", 0.10);
  const double odom_rot = declare_parameter<double>("graph_odom_rotation_sigma", 0.03);
  const double loop_trans = declare_parameter<double>("graph_loop_translation_sigma", 0.25);
  const double loop_rot = declare_parameter<double>("graph_loop_rotation_sigma", 0.05);
  const double huber_k = declare_parameter<double>("graph_loop_huber_k", 1.5);
  prior_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
      poseSigmas(std::max(1e-6, prior_rot), std::max(1e-6, prior_trans)));
  odom_noise_ = gtsam::noiseModel::Diagonal::Sigmas(
      poseSigmas(std::max(1e-6, odom_rot), std::max(1e-6, odom_trans)));
  loop_noise_ = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(std::max(0.01, huber_k)),
      gtsam::noiseModel::Diagonal::Sigmas(
          poseSigmas(std::max(1e-6, loop_rot), std::max(1e-6, loop_trans))));

  keyframe_cloud_max_points_ = std::max(100, keyframe_cloud_max_points_);
  if (!std::isfinite(icp_voxel_size_) || icp_voxel_size_ < 0.01) {
    icp_voxel_size_ = 0.5;
  }
  if (!std::isfinite(icp_max_correspondence_distance_) ||
      icp_max_correspondence_distance_ <= 0.0) {
    icp_max_correspondence_distance_ = 2.0;
  }
  if (!std::isfinite(icp_fitness_threshold_) || icp_fitness_threshold_ <= 0.0) {
    icp_fitness_threshold_ = 0.8;
  }
  icp_min_overlap_ratio_ = std::isfinite(icp_min_overlap_ratio_)
      ? std::clamp(icp_min_overlap_ratio_, 0.0, 1.0) : 0.35;
  if (!std::isfinite(icp_max_translation_correction_) ||
      icp_max_translation_correction_ <= 0.0) {
    icp_max_translation_correction_ = 5.0;
  }
  if (!std::isfinite(icp_max_rotation_correction_deg_) ||
      icp_max_rotation_correction_deg_ <= 0.0) {
    icp_max_rotation_correction_deg_ = 20.0;
  }
  icp_max_source_points_ = std::max(100, icp_max_source_points_);
  icp_max_target_points_ = std::max(100, icp_max_target_points_);
  icp_min_points_ = std::max(10, icp_min_points_);
  icp_submap_half_width_ = std::max(0, icp_submap_half_width_);
  icp_max_iterations_ = std::max(1, icp_max_iterations_);
  loop_min_interval_ = std::max(0, loop_min_interval_);
  map_publish_stride_ = std::max(1, map_publish_stride_);
  map_max_points_per_keyframe_ = std::max(1, map_max_points_per_keyframe_);
  path_max_poses_ = std::max(1, path_max_poses_);
  std_visualization_max_triangles_ =
      std::clamp(std_visualization_max_triangles_, 0, 1000);
  if (!std::isfinite(tf_publish_rate_hz_)) tf_publish_rate_hz_ = 20.0;

  auto output_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/lio_slam/odom", 10);
  path_pub_ = create_publisher<nav_msgs::msg::Path>("/lio_slam/path", output_qos);
  map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/lio_slam/map", output_qos);
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/lio_slam/loop_markers", output_qos);
  std_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/lio_slam/std_triangles", output_qos);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
  tf_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  tf_timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 /
          std::clamp(tf_publish_rate_hz_, 1.0, 100.0))),
      [this]() {
        publishMapToWorld(now());
      },
      tf_callback_group_);
  keyframe_sub_ = create_subscription<super_lio_slam::msg::KeyFrame>(
      "/lio_slam/keyframe", rclcpp::QoS(rclcpp::KeepLast(200)).reliable(),
      std::bind(&LoopBackendNode::onKeyFrame, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(), "STD, ICP and iSAM2 loop backend ready");
}

gtsam::Pose3 LoopBackendNode::poseFromMsg(const geometry_msgs::msg::Pose &pose) {
  const auto &p = pose.position;
  const auto &r = pose.orientation;
  if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(r.x) || !std::isfinite(r.y) || !std::isfinite(r.z) ||
      !std::isfinite(r.w)) {
    throw std::invalid_argument("non-finite keyframe pose");
  }
  Eigen::Quaterniond q(r.w, r.x, r.y, r.z);
  if (q.norm() < 1e-6) {
    throw std::invalid_argument("zero keyframe quaternion");
  }
  q.normalize();
  return gtsam::Pose3(gtsam::Rot3(q.w(), q.x(), q.y(), q.z()),
                      gtsam::Point3(p.x, p.y, p.z));
}

geometry_msgs::msg::Pose LoopBackendNode::poseToMsg(const gtsam::Pose3 &pose) {
  geometry_msgs::msg::Pose output;
  const auto p = pose.translation();
  const auto q = pose.rotation().toQuaternion();
  output.position.x = p.x();
  output.position.y = p.y();
  output.position.z = p.z();
  output.orientation.x = q.x();
  output.orientation.y = q.y();
  output.orientation.z = q.z();
  output.orientation.w = q.w();
  return output;
}

LoopBackendNode::CloudPtr LoopBackendNode::downsample(
    const CloudPtr &cloud, double voxel_size, std::size_t max_points) const {
  CloudPtr filtered(new Cloud);
  if (!cloud || cloud->empty()) return filtered;
  if (voxel_size > 0.01) {
    pcl::VoxelGrid<pcl::PointXYZI> voxel;
    voxel.setLeafSize(static_cast<float>(voxel_size),
                      static_cast<float>(voxel_size),
                      static_cast<float>(voxel_size));
    voxel.setInputCloud(cloud);
    voxel.filter(*filtered);
  } else {
    *filtered = *cloud;
  }
  if (max_points > 0 && filtered->size() > max_points) {
    CloudPtr capped(new Cloud);
    capped->reserve(max_points);
    const std::size_t stride =
        (filtered->size() + max_points - 1) / max_points;
    for (std::size_t i = 0; i < filtered->size(); i += stride) {
      capped->push_back((*filtered)[i]);
    }
    return capped;
  }
  return filtered;
}

LoopBackendNode::CloudPtr LoopBackendNode::convertAndFilter(
    const sensor_msgs::msg::PointCloud2 &message) const {
  CloudPtr raw(new Cloud);
  pcl::fromROSMsg(message, *raw);
  CloudPtr finite(new Cloud);
  finite->reserve(raw->size());
  for (auto p : *raw) {
    if (std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z)) {
      if (!std::isfinite(p.intensity)) p.intensity = 0.0f;
      finite->push_back(p);
    }
  }
  return downsample(finite, keyframe_cloud_voxel_size_,
                    static_cast<std::size_t>(keyframe_cloud_max_points_));
}

LoopBackendNode::CloudPtr LoopBackendNode::historicalSubmap(
    std::size_t historical_index) const {
  CloudPtr submap(new Cloud);
  if (historical_index >= frames_.size()) return submap;
  const std::size_t half = static_cast<std::size_t>(icp_submap_half_width_);
  const std::size_t first = historical_index > half ? historical_index - half : 0;
  const std::size_t last = std::min(frames_.size() - 1, historical_index + half);
  const std::size_t count = last - first + 1;
  const std::size_t per_frame_cap = std::max<std::size_t>(
      100, (2 * static_cast<std::size_t>(icp_max_target_points_)) / count);
  const gtsam::Pose3 &hist_pose = frames_[historical_index].odom_pose;
  for (std::size_t i = first; i <= last; ++i) {
    if (!frames_[i].cloud || frames_[i].cloud->empty()) continue;
    const CloudPtr local = downsample(frames_[i].cloud, icp_voxel_size_,
                                      per_frame_cap);
    Cloud transformed;
    const Eigen::Matrix4f historical_from_neighbor =
        hist_pose.between(frames_[i].odom_pose).matrix().cast<float>();
    pcl::transformPointCloud(*local, transformed, historical_from_neighbor);
    *submap += transformed;
  }
  return downsample(submap, icp_voxel_size_,
                    static_cast<std::size_t>(icp_max_target_points_));
}

std::unique_ptr<LoopBackendNode::IcpObservation> LoopBackendNode::verifyWithIcp(
    const CloudPtr &current_cloud, std::size_t historical_index,
    const std::pair<Eigen::Vector3d, Eigen::Matrix3d> &std_transform) const {
  if (historical_index >= frames_.size() || !current_cloud ||
      !std_transform.first.allFinite() || !std_transform.second.allFinite()) {
    return nullptr;
  }
  const CloudPtr source = downsample(
      current_cloud, icp_voxel_size_,
      static_cast<std::size_t>(icp_max_source_points_));
  const CloudPtr target = historicalSubmap(historical_index);
  if (source->size() < static_cast<std::size_t>(icp_min_points_) ||
      target->size() < static_cast<std::size_t>(icp_min_points_)) {
    RCLCPP_INFO(get_logger(), "ICP rejected: insufficient points (%zu source, %zu target, need %d)",
                source->size(), target->size(), icp_min_points_);
    return nullptr;
  }

  Eigen::Matrix4f initial = Eigen::Matrix4f::Identity();
  initial.block<3, 3>(0, 0) = std_transform.second.cast<float>();
  initial.block<3, 1>(0, 3) = std_transform.first.cast<float>();
  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setInputSource(source);
  icp.setInputTarget(target);
  icp.setMaximumIterations(icp_max_iterations_);
  icp.setMaxCorrespondenceDistance(icp_max_correspondence_distance_);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  Cloud aligned;
  icp.align(aligned, initial);
  if (aligned.empty() || !icp.hasConverged() ||
      !std::isfinite(icp.getFitnessScore()) ||
      icp.getFitnessScore() > icp_fitness_threshold_) {
    RCLCPP_INFO(get_logger(), "ICP rejected: converged=%d, aligned=%zu, fitness=%.3f (limit %.3f)",
                icp.hasConverged(), aligned.size(), icp.getFitnessScore(), icp_fitness_threshold_);
    return nullptr;
  }
  const Eigen::Matrix4f result = icp.getFinalTransformation();
  if (!result.allFinite()) return nullptr;
  const Eigen::Matrix4d correction =
      initial.cast<double>().inverse() * result.cast<double>();
  const double correction_translation = correction.block<3, 1>(0, 3).norm();
  const double correction_rotation = degrees(
      Eigen::AngleAxisd(correction.block<3, 3>(0, 0)).angle());
  if (correction_translation > icp_max_translation_correction_ ||
      correction_rotation > icp_max_rotation_correction_deg_) {
    RCLCPP_INFO(get_logger(), "ICP rejected: correction %.3f m, %.2f deg (limits %.3f m, %.2f deg)",
                correction_translation, correction_rotation,
                icp_max_translation_correction_, icp_max_rotation_correction_deg_);
    return nullptr;
  }

  pcl::KdTreeFLANN<pcl::PointXYZI> tree;
  tree.setInputCloud(target);
  std::vector<int> indices(1);
  std::vector<float> squared_distances(1);
  std::size_t correspondences = 0;
  const double maximum_squared = icp_max_correspondence_distance_ *
                                 icp_max_correspondence_distance_;
  for (const auto &point : aligned) {
    if (tree.nearestKSearch(point, 1, indices, squared_distances) == 1 &&
        squared_distances.front() <= maximum_squared) {
      ++correspondences;
    }
  }
  const double overlap = static_cast<double>(correspondences) /
                         static_cast<double>(aligned.size());
  if (!std::isfinite(overlap) || overlap < icp_min_overlap_ratio_) {
    RCLCPP_INFO(get_logger(), "ICP rejected: overlap %.3f (minimum %.3f)",
                overlap, icp_min_overlap_ratio_);
    return nullptr;
  }

  const Eigen::Matrix3d rotation = result.block<3, 3>(0, 0).cast<double>();
  Eigen::Quaterniond q(rotation);
  q.normalize();
  auto observation = std::make_unique<IcpObservation>();
  observation->historical_from_current = gtsam::Pose3(
      gtsam::Rot3(q.w(), q.x(), q.y(), q.z()),
      gtsam::Point3(result(0, 3), result(1, 3), result(2, 3)));
  observation->fitness = icp.getFitnessScore();
  observation->overlap = overlap;
  return observation;
}

void LoopBackendNode::addGraphNode(
    std::size_t index, const gtsam::Pose3 &raw_pose,
    const std::unique_ptr<IcpObservation> &loop,
    std::size_t historical_index) {
  gtsam::NonlinearFactorGraph factors;
  gtsam::Values initial;
  if (index == 0) {
    factors.add(gtsam::PriorFactor<gtsam::Pose3>(0, raw_pose, prior_noise_));
    initial.insert(0, raw_pose);
  } else {
    const auto raw_delta = frames_[index - 1].odom_pose.between(raw_pose);
    factors.add(gtsam::BetweenFactor<gtsam::Pose3>(index - 1, index,
                                                  raw_delta, odom_noise_));
    const gtsam::Pose3 seed = loop
        ? frames_[historical_index].map_pose.compose(
              loop->historical_from_current)
        : frames_[index - 1].map_pose.compose(raw_delta);
    initial.insert(index, seed);
  }
  if (loop) {
    factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
        historical_index, index, loop->historical_from_current, loop_noise_));
  }
  isam_.update(factors, initial);
  if (loop) {
    // A loop can move every historical pose; refresh all published geometry.
    isam_.update();
    const gtsam::Values estimate = isam_.calculateEstimate();
    for (std::size_t i = 0; i < frames_.size(); ++i) {
      frames_[i].map_pose = estimate.at<gtsam::Pose3>(i);
    }
  } else {
    frames_[index].map_pose = isam_.calculateEstimate<gtsam::Pose3>(index);
  }
  {
    std::lock_guard<std::mutex> lock(map_tf_mutex_);
    map_from_world_ = frames_[index].map_pose.compose(raw_pose.inverse());
    have_map_transform_ = true;
  }
}

void LoopBackendNode::onKeyFrame(
    const super_lio_slam::msg::KeyFrame::SharedPtr msg) {
  if (backend_faulted_) return;
  if (have_input_id_ && msg->id <= last_input_id_) {
    RCLCPP_WARN(get_logger(), "Ignoring duplicate or out-of-order keyframe %u",
                msg->id);
    return;
  }
  gtsam::Pose3 raw_pose;
  CloudPtr cloud;
  try {
    raw_pose = poseFromMsg(msg->odom_pose);
    cloud = convertAndFilter(msg->cloud);
  } catch (const std::exception &error) {
    RCLCPP_WARN(get_logger(), "Ignoring keyframe %u: %s", msg->id,
                error.what());
    return;
  }
  if (cloud->size() < 10) {
    RCLCPP_WARN(get_logger(), "Ignoring sparse keyframe %u (%zu points)",
                msg->id, cloud->size());
    return;
  }
  if (have_input_id_ && msg->id != last_input_id_ + 1) {
    RCLCPP_WARN(get_logger(), "Keyframe ID gap: %u followed by %u; using dense backend indices",
                last_input_id_, msg->id);
  }

  const std::size_t index = frames_.size();
  // GenerateSTDescs appends this frame's plane cloud. Search must run before
  // AddSTDescs so a frame cannot match itself, and AddSTDescs advances the ID.
  std::vector<STDesc> descriptors;
  std::pair<int, double> search_result{-1, 0.0};
  std::pair<Eigen::Vector3d, Eigen::Matrix3d> std_transform;
  std::vector<std::pair<STDesc, STDesc>> matched_pairs;
  const bool enough_time_since_loop =
      !have_loop_ || index - last_loop_index_ >=
                         static_cast<std::size_t>(loop_min_interval_);
  const auto plane_count = std_manager_.plane_cloud_vec_.size();
  const auto corner_count = std_manager_.corner_cloud_vec_.size();
  bool adding_descriptors = false;
  try {
    std_manager_.GenerateSTDescs(cloud, descriptors);
    if (enough_time_since_loop && !descriptors.empty() &&
        !std_manager_.plane_cloud_vec_.empty() &&
        std_manager_.plane_cloud_vec_.back()->size() >= 3) {
      std_manager_.SearchLoop(descriptors, search_result, std_transform,
                              matched_pairs);
    }
    adding_descriptors = true;
    std_manager_.AddSTDescs(descriptors);
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_logger(), "STD failed on keyframe %u: %s", msg->id,
                 error.what());
    if (adding_descriptors) {
      // Insertion into STD's hash table is not transactional.
      backend_faulted_ = true;
      std::lock_guard<std::mutex> lock(map_tf_mutex_);
      have_map_transform_ = false;
    } else {
      std_manager_.plane_cloud_vec_.resize(plane_count);
      std_manager_.corner_cloud_vec_.resize(corner_count);
    }
    return;
  }

  std::size_t historical_index = 0;
  std::unique_ptr<IcpObservation> loop;
  if (search_result.first >= 0 &&
      static_cast<std::size_t>(search_result.first) < index) {
    historical_index = static_cast<std::size_t>(search_result.first);
    try {
      loop = verifyWithIcp(cloud, historical_index, std_transform);
    } catch (const std::exception &error) {
      RCLCPP_WARN(get_logger(), "ICP rejected keyframe %u: %s", msg->id,
                  error.what());
    }
    if (loop) {
      RCLCPP_INFO(get_logger(),
                  "Loop %zu -> %zu accepted: STD %.3f, ICP fitness %.3f, overlap %.2f",
                  historical_index, index, search_result.second,
                  loop->fitness, loop->overlap);
    } else {
      RCLCPP_INFO(get_logger(), "Loop candidate %zu -> %zu rejected by ICP",
                  historical_index, index);
    }
  }

  Frame frame;
  frame.input_id = msg->id;
  frame.stamp = msg->header.stamp;
  frame.odom_pose = raw_pose;
  frame.cloud = std::move(cloud);
  frames_.push_back(std::move(frame));
  try {
    addGraphNode(index, raw_pose, loop, historical_index);
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_logger(), "iSAM2 update failed on keyframe %u: %s",
                 msg->id, error.what());
    backend_faulted_ = true;
    std::lock_guard<std::mutex> lock(map_tf_mutex_);
    have_map_transform_ = false;
    return;
  }
  if (loop) {
    loops_.emplace_back(historical_index, index);
    last_loop_index_ = index;
    have_loop_ = true;
  }
  try {
    publishCurrent(index, static_cast<bool>(loop));
    publishStdTriangles(index, descriptors);
  } catch (const std::exception &error) {
    RCLCPP_ERROR(get_logger(), "Unable to publish keyframe %u: %s", msg->id,
                 error.what());
  }
  last_input_id_ = msg->id;
  have_input_id_ = true;
}

void LoopBackendNode::publishCurrent(std::size_t index, bool loop_accepted) {
  const Frame &frame = frames_[index];
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = frame.stamp;
  odom.header.frame_id = "map";
  odom.child_frame_id = "imu";
  odom.pose.pose = poseToMsg(frame.map_pose);
  odom_pub_->publish(odom);

  publishMapToWorld(frame.stamp);

  publishPath(frame.stamp);
  if (loop_accepted || index % static_cast<std::size_t>(map_publish_stride_) == 0) {
    publishMap(frame.stamp);
  }
  if (loop_accepted) publishLoopMarkers(frame.stamp);
}

void LoopBackendNode::publishMapToWorld(
    const builtin_interfaces::msg::Time &stamp) {
  gtsam::Pose3 map_from_world;
  {
    std::lock_guard<std::mutex> lock(map_tf_mutex_);
    if (!have_map_transform_) return;
    map_from_world = map_from_world_;
  }
  geometry_msgs::msg::TransformStamped tf;
  tf.header.stamp = stamp;
  tf.header.frame_id = "map";
  tf.child_frame_id = "world";
  const auto transform = poseToMsg(map_from_world);
  tf.transform.translation.x = transform.position.x;
  tf.transform.translation.y = transform.position.y;
  tf.transform.translation.z = transform.position.z;
  tf.transform.rotation = transform.orientation;
  tf_broadcaster_->sendTransform(tf);
}

void LoopBackendNode::publishPath(const builtin_interfaces::msg::Time &stamp) {
  nav_msgs::msg::Path path;
  path.header.stamp = stamp;
  path.header.frame_id = "map";
  const std::size_t first = frames_.size() > static_cast<std::size_t>(path_max_poses_)
                                ? frames_.size() - static_cast<std::size_t>(path_max_poses_)
                                : 0;
  path.poses.reserve(frames_.size() - first);
  for (std::size_t i = first; i < frames_.size(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = frames_[i].stamp;
    pose.pose = poseToMsg(frames_[i].map_pose);
    path.poses.push_back(std::move(pose));
  }
  path_pub_->publish(path);
}

void LoopBackendNode::publishMap(const builtin_interfaces::msg::Time &stamp) {
  CloudPtr merged(new Cloud);
  const std::size_t limit = map_max_keyframes_ > 0
      ? static_cast<std::size_t>(map_max_keyframes_) : frames_.size();
  const std::size_t first = frames_.size() > limit ? frames_.size() - limit : 0;
  for (std::size_t i = first; i < frames_.size(); ++i) {
    const CloudPtr sampled = downsample(
        frames_[i].cloud, 0.0,
        static_cast<std::size_t>(map_max_points_per_keyframe_));
    if (sampled->empty()) continue;
    Cloud transformed;
    pcl::transformPointCloud(*sampled, transformed,
                             frames_[i].map_pose.matrix().cast<float>());
    *merged += transformed;
  }
  const CloudPtr map = downsample(merged, map_voxel_size_, 0);
  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(*map, output);
  output.header.stamp = stamp;
  output.header.frame_id = "map";
  map_pub_->publish(output);
}

void LoopBackendNode::publishStdTriangles(
    std::size_t index, const std::vector<STDesc> &descriptors) {
  using Marker = visualization_msgs::msg::Marker;
  Marker edges;
  edges.header.frame_id = "map";
  edges.header.stamp = frames_[index].stamp;
  edges.ns = "std_triangle_edges";
  edges.id = 0;
  edges.type = Marker::LINE_LIST;
  edges.action = Marker::ADD;
  edges.pose.orientation.w = 1.0;
  edges.scale.x = 0.08;
  edges.color.r = 0.1f;
  edges.color.g = 0.7f;
  edges.color.b = 1.0f;
  edges.color.a = 0.9f;

  Marker vertices = edges;
  vertices.ns = "std_triangle_vertices";
  vertices.type = Marker::SPHERE_LIST;
  vertices.scale.x = vertices.scale.y = vertices.scale.z = 0.18;
  vertices.color.r = 1.0f;
  vertices.color.g = 0.9f;
  vertices.color.b = 0.2f;

  if (std_visualization_max_triangles_ > 0) {
    const std::size_t limit =
        static_cast<std::size_t>(std_visualization_max_triangles_);
    const std::size_t step = std::max<std::size_t>(
        1, (descriptors.size() + limit - 1) / limit);
    const auto &pose = frames_[index].map_pose;
    for (std::size_t i = 0; i < descriptors.size(); i += step) {
      const auto &descriptor = descriptors[i];
      const Eigen::Vector3d *local[3] = {
          &descriptor.vertex_A_, &descriptor.vertex_B_, &descriptor.vertex_C_};
      if (!local[0]->allFinite() || !local[1]->allFinite() ||
          !local[2]->allFinite()) continue;
      geometry_msgs::msg::Point corners[3];
      for (int j = 0; j < 3; ++j) {
        const Eigen::Vector3d point =
            pose.rotation().matrix() * *local[j] + pose.translation();
        corners[j].x = point.x();
        corners[j].y = point.y();
        corners[j].z = point.z();
        vertices.points.push_back(corners[j]);
      }
      for (int j = 0; j < 3; ++j) {
        edges.points.push_back(corners[j]);
        edges.points.push_back(corners[(j + 1) % 3]);
      }
    }
  }
  // Reuse fixed marker IDs so each keyframe replaces the previous triangles.
  // DELETE also clears an older display when this frame has no descriptors.
  if (edges.points.empty()) {
    edges.action = Marker::DELETE;
    vertices.action = Marker::DELETE;
  }
  visualization_msgs::msg::MarkerArray markers;
  markers.markers.push_back(std::move(edges));
  markers.markers.push_back(std::move(vertices));
  std_marker_pub_->publish(markers);
}

void LoopBackendNode::publishLoopMarkers(
    const builtin_interfaces::msg::Time &stamp) {
  visualization_msgs::msg::Marker line;
  line.header.frame_id = "map";
  line.header.stamp = stamp;
  line.ns = "accepted_loops";
  line.id = 0;
  line.type = visualization_msgs::msg::Marker::LINE_LIST;
  line.action = visualization_msgs::msg::Marker::ADD;
  line.pose.orientation.w = 1.0;
  line.scale.x = 0.15;
  line.color.r = 0.0f;
  line.color.g = 0.9f;
  line.color.b = 0.3f;
  line.color.a = 1.0f;
  line.points.reserve(2 * loops_.size());
  for (const auto &[historical, current] : loops_) {
    for (const auto index : {historical, current}) {
      const auto position = frames_[index].map_pose.translation();
      geometry_msgs::msg::Point point;
      point.x = position.x();
      point.y = position.y();
      point.z = position.z();
      line.points.push_back(point);
    }
  }
  visualization_msgs::msg::MarkerArray markers;
  markers.markers.push_back(std::move(line));
  marker_pub_->publish(markers);
}

}  // namespace super_lio_slam
