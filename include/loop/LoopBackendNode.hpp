#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <utility>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "STDesc.h"
#include "super_lio_slam/msg/key_frame.hpp"

namespace super_lio_slam {

class LoopBackendNode final : public rclcpp::Node {
 public:
  LoopBackendNode();

 private:
  using Cloud = pcl::PointCloud<pcl::PointXYZI>;
  using CloudPtr = Cloud::Ptr;

  struct Frame {
    uint32_t input_id = 0;
    builtin_interfaces::msg::Time stamp;
    gtsam::Pose3 odom_pose;
    gtsam::Pose3 map_pose;
    CloudPtr cloud;
  };

  struct IcpObservation {
    gtsam::Pose3 historical_from_current;
    double fitness = 0.0;
    double overlap = 0.0;
  };

  void onKeyFrame(const super_lio_slam::msg::KeyFrame::SharedPtr msg);
  CloudPtr convertAndFilter(const sensor_msgs::msg::PointCloud2 &message) const;
  CloudPtr downsample(const CloudPtr &cloud, double voxel_size,
                      std::size_t max_points) const;
  CloudPtr historicalSubmap(std::size_t historical_index) const;
  CloudPtr descriptorSubmap(const CloudPtr &current_cloud,
                            const gtsam::Pose3 &current_pose) const;
  std::unique_ptr<IcpObservation> verifyWithIcp(
      const CloudPtr &current_cloud, std::size_t current_index,
      std::size_t historical_index,
      const std::pair<Eigen::Vector3d, Eigen::Matrix3d> &std_transform) const;

  void addGraphNode(std::size_t index, const gtsam::Pose3 &raw_pose,
                    const std::unique_ptr<IcpObservation> &loop,
                    std::size_t historical_index);
  void publishCurrent(std::size_t index, bool loop_accepted);
  void publishMapToWorld(const builtin_interfaces::msg::Time &stamp);
  void publishPath(const builtin_interfaces::msg::Time &stamp);
  void publishMap(const builtin_interfaces::msg::Time &stamp);
  void publishLoopMarkers(const builtin_interfaces::msg::Time &stamp);
  void publishStdTriangles(std::size_t index,
                           const std::vector<STDesc> &descriptors);

  static gtsam::Pose3 poseFromMsg(const geometry_msgs::msg::Pose &pose);
  static geometry_msgs::msg::Pose poseToMsg(const gtsam::Pose3 &pose);

  STDescManager std_manager_;
  std::vector<Frame> frames_;
  std::vector<std::pair<std::size_t, std::size_t>> loops_;
  gtsam::ISAM2 isam_;
  gtsam::noiseModel::Diagonal::shared_ptr prior_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr odom_noise_;
  gtsam::noiseModel::Base::shared_ptr loop_noise_;
  gtsam::Pose3 map_from_world_;
  mutable std::mutex map_tf_mutex_;
  bool have_map_transform_ = false;

  rclcpp::Subscription<super_lio_slam::msg::KeyFrame>::SharedPtr keyframe_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr std_marker_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::CallbackGroup::SharedPtr tf_callback_group_;
  rclcpp::TimerBase::SharedPtr tf_timer_;

  double keyframe_cloud_voxel_size_ = 0.25;
  int keyframe_cloud_max_points_ = 30000;
  double icp_voxel_size_ = 0.5;
  double icp_max_correspondence_distance_ = 2.0;
  int icp_max_iterations_ = 50;
  double icp_fitness_threshold_ = 0.8;
  double icp_min_overlap_ratio_ = 0.35;
  double icp_max_translation_correction_ = 3.0;
  double icp_max_rotation_correction_deg_ = 20.0;
  int icp_submap_half_width_ = 5;
  int icp_min_points_ = 100;
  int icp_max_source_points_ = 10000;
  int icp_max_target_points_ = 50000;
  int loop_min_interval_ = 20;
  int std_submap_num_frames_ = 1;
  double std_submap_max_distance_ = 6.0;
  int icp_max_std_candidates_ = 3;
  int loop_min_descriptor_inliers_ = 4;
  int loop_diagnostic_stride_ = 1;
  int map_publish_stride_ = 30;
  int map_max_keyframes_ = 2000;
  int map_max_points_per_keyframe_ = 1000;
  double map_voxel_size_ = 0.5;
  int path_max_poses_ = 10000;
  int std_visualization_max_triangles_ = 100;
  double tf_publish_rate_hz_ = 20.0;
  std::size_t last_loop_index_ = 0;
  std::size_t last_loop_historical_index_ = 0;
  bool have_loop_ = false;
  bool have_input_id_ = false;
  uint32_t last_input_id_ = 0;
  bool backend_faulted_ = false;
};

}  // namespace super_lio_slam
