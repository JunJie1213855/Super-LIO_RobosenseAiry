#include "STDesc.h"
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <functional>

class StdExtractNode : public rclcpp::Node {
public:
  StdExtractNode() : Node("std_extract_node") {
    const auto topic = declare_parameter<std::string>("input_topic", "/points_raw");
    output_dir_ = declare_parameter<std::string>("output_dir", "/tmp/std_descriptors");
    max_visualized_ = std::max<int>(0, declare_parameter<int>("max_visualized_descriptors", 100));
    ConfigSetting config;
    read_parameters(*this, config);
    manager_ = std::make_unique<STDescManager>(config);
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/std/descriptors", 10);
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        topic, rclcpp::SensorDataQoS(),
        std::bind(&StdExtractNode::onCloud, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "STD extraction: %s -> %s", topic.c_str(), output_dir_.c_str());
  }
private:
  void publishVisualization(const std_msgs::msg::Header &header,
                            const std::vector<STDesc> &descriptors) {
    using Marker = visualization_msgs::msg::Marker;
    visualization_msgs::msg::MarkerArray array;
    Marker clear;
    clear.action = Marker::DELETEALL;
    array.markers.push_back(clear);

    Marker edges;
    edges.header = header;
    edges.ns = "std_triangles";
    edges.id = 0;
    edges.type = Marker::LINE_LIST;
    edges.action = Marker::ADD;
    edges.pose.orientation.w = 1.0;
    edges.scale.x = 0.08;
    edges.color.r = 0.1f;
    edges.color.g = 0.7f;
    edges.color.b = 1.0f;
    edges.color.a = 0.85f;

    Marker vertices = edges;
    vertices.ns = "std_vertices";
    vertices.type = Marker::SPHERE_LIST;
    vertices.scale.x = vertices.scale.y = vertices.scale.z = 0.18;
    vertices.color.r = 1.0f;
    vertices.color.g = 0.9f;
    vertices.color.b = 0.2f;

    if (max_visualized_ > 0) {
      const size_t step = std::max<size_t>(
          1, (descriptors.size() + max_visualized_ - 1) / max_visualized_);
      for (size_t i = 0; i < descriptors.size(); i += step) {
        const auto &d = descriptors[i];
        geometry_msgs::msg::Point points[3];
        const Eigen::Vector3d *vectors[3] = {&d.vertex_A_, &d.vertex_B_, &d.vertex_C_};
        for (int j = 0; j < 3; ++j) {
          points[j].x = (*vectors[j]).x();
          points[j].y = (*vectors[j]).y();
          points[j].z = (*vectors[j]).z();
          vertices.points.push_back(points[j]);
        }
        for (int j = 0; j < 3; ++j) {
          edges.points.push_back(points[j]);
          edges.points.push_back(points[(j + 1) % 3]);
        }
      }
    }
    array.markers.push_back(edges);
    array.markers.push_back(vertices);
    marker_publisher_->publish(array);
  }
  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    auto cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    try {
      pcl::fromROSMsg(*msg, *cloud);
      if (cloud->empty()) return;
      const auto frame_id = sequence_++;
      manager_->current_frame_id_ = static_cast<unsigned int>(frame_id);
      std::vector<STDesc> descriptors;
      manager_->GenerateSTDescs(cloud, descriptors);
      publishVisualization(msg->header, descriptors);
      std::filesystem::create_directories(output_dir_);
      const auto path = std::filesystem::path(output_dir_) /
          ("std_" + std::to_string(frame_id) + ".csv");
      std::ofstream file(path);
      if (!file) throw std::runtime_error("cannot open output file");
      file << "frame_id,side_a,side_b,side_c,angle_a,angle_b,angle_c,center_x,center_y,center_z,ax,ay,az,bx,by,bz,cx,cy,cz,attached_a,attached_b,attached_c\n";
      file << std::setprecision(17);
      for (const auto &d : descriptors) {
        file << d.frame_id_;
        for (const auto *v : {&d.side_length_, &d.angle_, &d.center_,
                              &d.vertex_A_, &d.vertex_B_, &d.vertex_C_,
                              &d.vertex_attached_})
          for (int i = 0; i < 3; ++i) file << ',' << (*v)[i];
        file << '\n';
      }
      RCLCPP_INFO(get_logger(), "Wrote %zu STD descriptors: %s", descriptors.size(), path.c_str());
      manager_->plane_cloud_vec_.clear();
      manager_->corner_cloud_vec_.clear();
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "STD extraction failed: %s", e.what());
    }
  }
  std::string output_dir_;
  int max_visualized_ = 100;
  size_t sequence_ = 0;
  std::unique_ptr<STDescManager> manager_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StdExtractNode>());
  rclcpp::shutdown();
  return 0;
}
