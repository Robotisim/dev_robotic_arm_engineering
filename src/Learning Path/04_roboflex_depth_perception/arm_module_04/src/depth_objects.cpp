#include "arm_course/ros_helpers.hpp"
#include <arm_course_interfaces/msg/objects.hpp>
#include <map>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <set>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

class DepthObjects : public rclcpp::Node {
  tf2_ros::Buffer tf_;
  tf2_ros::TransformListener listener_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr stage_;
  rclcpp::Publisher<arm_course_interfaces::msg::Objects>::SharedPtr objects_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr scene_;
  std::string base_, cloud_frame_;
  std::vector<double> roi_;
  double table_, voxel_, cube_height_;
  uint64_t revision_{0};
  bool frozen_{false};
  std::set<std::string> previous_;
  std::map<std::string, int> missed_;
  void process(sensor_msgs::msg::PointCloud2::ConstSharedPtr message) {
    try {
      // Harmonic RGBD points use camera-link XYZ (X forward), even when
      // gz_frame_id labels the image/CameraInfo optical frame.
      auto transform = tf_.lookupTransform(
          base_, cloud_frame_.empty() ? message->header.frame_id : cloud_frame_,
          message->header.stamp, rclcpp::Duration::from_seconds(0.1));
      Eigen::Quaternionf rotation(
          transform.transform.rotation.w, transform.transform.rotation.x,
          transform.transform.rotation.y, transform.transform.rotation.z);
      Eigen::Vector3f offset(transform.transform.translation.x,
                             transform.transform.translation.y,
                             transform.transform.translation.z);
      pcl::PointCloud<pcl::PointXYZRGB> raw;
      pcl::fromROSMsg(*message, raw);
      std::map<std::string, pcl::PointCloud<pcl::PointXYZRGB>::Ptr> by_colour;
      for (auto colour : {"red", "green", "blue"})
        by_colour[colour] =
            std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
      for (auto p : raw) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
          continue;
        Eigen::Vector3f xyz =
            rotation * Eigen::Vector3f(p.x, p.y, p.z) + offset;
        p.x = xyz.x();
        p.y = xyz.y();
        p.z = xyz.z();
        if (p.x < roi_[0] || p.x > roi_[1] || p.y < roi_[2] || p.y > roi_[3] ||
            p.z < table_ + 0.007 || p.z > roi_[5])
          continue;
        std::string colour;
        if (p.r > 60 && p.r > p.g * 1.5 && p.r > p.b * 1.5)
          colour = "red";
        if (p.g > 60 && p.g > p.r * 1.5 && p.g > p.b * 1.5)
          colour = "green";
        if (p.b > 60 && p.b > p.r * 1.5 && p.b > p.g * 1.5)
          colour = "blue";
        if (!colour.empty())
          by_colour[colour]->push_back(p);
      }
      arm_course_interfaces::msg::Objects out;
      out.header = message->header;
      out.header.frame_id = base_;
      out.revision = ++revision_;
      pcl::PointCloud<pcl::PointXYZRGB> clean;
      for (auto &[colour, cloud] : by_colour) {
        if (cloud->empty())
          continue;
        auto down = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        pcl::VoxelGrid<pcl::PointXYZRGB> filter;
        filter.setInputCloud(cloud);
        filter.setLeafSize(voxel_, voxel_, voxel_);
        filter.filter(*down);
        clean += *down;
        auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>();
        tree->setInputCloud(down);
        pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> cluster;
        cluster.setClusterTolerance(0.012);
        cluster.setMinClusterSize(8);
        cluster.setMaxClusterSize(3000);
        cluster.setSearchMethod(tree);
        cluster.setInputCloud(down);
        std::vector<pcl::PointIndices> groups;
        cluster.extract(groups);
        int index = 0;
        for (const auto &indices : groups) {
          Eigen::Vector3f low = Eigen::Vector3f::Constant(1e6), high = -low;
          for (int i : indices.indices) {
            const auto &p = (*down)[i];
            Eigen::Vector3f v(p.x, p.y, p.z);
            low = low.cwiseMin(v);
            high = high.cwiseMax(v);
          }
          if (high.x() - low.x() > 0.09 || high.y() - low.y() > 0.09)
            continue;
          // For this cube lab, require a visible top patch. Thin side-face
          // fragments must not become a second object with the same colour.
          if (high.x() - low.x() < 0.025 || high.y() - low.y() < 0.025 ||
              high.z() - table_ < 0.025)
            continue;
          arm_course_interfaces::msg::Object object;
          object.id =
              colour + "_cube" + (index ? "_" + std::to_string(index) : "");
          ++index;
          object.colour = colour;
          object.size.x = std::max(0.02f, high.x() - low.x() + float(voxel_));
          object.size.y = std::max(0.02f, high.y() - low.y() + float(voxel_));
          // Known cube height disambiguates an elevated cube from a tall box.
          // RGBD supplies its top surface, including when it sits on a stack.
          object.size.z = cube_height_;
          object.pose.position.x = (low.x() + high.x()) / 2;
          object.pose.position.y = (low.y() + high.y()) / 2;
          object.pose.position.z = high.z() - cube_height_ / 2;
          object.pose.orientation.w = 1;
          object.confidence = std::min(1.0, indices.indices.size() / 50.0);
          out.objects.push_back(object);
        }
      }
      objects_->publish(out);
      sensor_msgs::msg::PointCloud2 filtered;
      pcl::toROSMsg(clean, filtered);
      filtered.header = out.header;
      filtered_->publish(filtered);
      visualization_msgs::msg::MarkerArray markers;
      visualization_msgs::msg::Marker erase;
      erase.action = erase.DELETEALL;
      markers.markers.push_back(erase);
      int id = 0;
      for (const auto &o : out.objects) {
        visualization_msgs::msg::Marker m;
        m.header = out.header;
        m.ns = "detected_cubes";
        m.id = id++;
        m.type = m.CUBE;
        m.action = m.ADD;
        m.pose = o.pose;
        m.scale = o.size;
        m.color.a = 0.5;
        m.color.r = o.colour == "red";
        m.color.g = o.colour == "green";
        m.color.b = o.colour == "blue";
        markers.markers.push_back(m);
      }
      markers_->publish(markers);
      if (!frozen_) {
        moveit_msgs::msg::PlanningScene scene;
        scene.is_diff = true;
        scene.robot_state.is_diff = true;
        std::set<std::string> present;
        for (const auto &o : out.objects) {
          moveit_msgs::msg::CollisionObject c;
          c.header = out.header;
          c.id = o.id;
          c.operation = c.ADD;
          shape_msgs::msg::SolidPrimitive box;
          box.type = box.BOX;
          box.dimensions = {o.size.x, o.size.y, o.size.z};
          c.primitives.push_back(box);
          c.primitive_poses.push_back(o.pose);
          scene.world.collision_objects.push_back(c);
          present.insert(o.id);
          missed_[o.id] = 0;
        }
        for (const auto &id : previous_)
          if (!present.count(id) && ++missed_[id] >= 20) {
            moveit_msgs::msg::CollisionObject c;
            c.id = id;
            c.operation = c.REMOVE;
            scene.world.collision_objects.push_back(c);
          } else if (!present.count(id)) {
            present.insert(id);
          }
        previous_ = present;
        scene_->publish(scene);
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Perception: %s",
                           e.what());
    }
  }

public:
  DepthObjects() : Node("depth_objects"), tf_(get_clock()), listener_(tf_) {
    base_ = declare_parameter<std::string>("base_frame", "panda_link0");
    cloud_frame_ = declare_parameter<std::string>("cloud_frame", "camera_link");
    roi_ = declare_parameter<std::vector<double>>(
        "roi", {0.25, 0.75, -0.4, 0.4, 0.0, 0.20});
    table_ = declare_parameter<double>("table_z", 0);
    voxel_ = declare_parameter<double>("voxel", 0.004);
    cube_height_ = declare_parameter<double>("cube_height", 0.04);
    if (roi_.size() != 6 || voxel_ <= 0)
      throw std::runtime_error("Invalid ROI/voxel configuration");
    objects_ = create_publisher<arm_course_interfaces::msg::Objects>(
        "/learning/objects", 10);
    filtered_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/learning/filtered_cloud", 10);
    markers_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/learning/object_markers", 10);
    scene_ = create_publisher<moveit_msgs::msg::PlanningScene>(
        "/planning_scene", 10);
    stage_ = create_subscription<std_msgs::msg::String>(
        "/learning/stage", rclcpp::QoS(1).transient_local(),
        [this](std_msgs::msg::String::ConstSharedPtr m) {
          frozen_ = m->data != "idle";
        });
    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/points", rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m) {
          process(m);
        });
  }
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<DepthObjects>());
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
}
