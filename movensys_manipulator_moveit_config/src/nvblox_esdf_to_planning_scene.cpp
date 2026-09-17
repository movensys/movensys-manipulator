#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <moveit_msgs/msg/planning_scene.hpp>
#include <nvblox_msgs/srv/esdf_and_gradients.hpp>
#include <octomap/OcTree.h>
#include <octomap_msgs/conversions.h>
#include <rclcpp/rclcpp.hpp>

namespace
{
constexpr float kNvbloxUnobservedValue = -1000.0F;

std::vector<double> declareVectorParameter(
  rclcpp::Node * node,
  const std::string & name,
  const std::vector<double> & default_value)
{
  const auto value = node->declare_parameter<std::vector<double>>(name, default_value);
  if (value.size() != default_value.size()) {
    RCLCPP_WARN(
      node->get_logger(),
      "Parameter '%s' must contain %zu values. Falling back to the default.",
      name.c_str(), default_value.size());
    return default_value;
  }
  return value;
}
}  // namespace

class NvbloxEsdfToPlanningScene : public rclcpp::Node
{
public:
  NvbloxEsdfToPlanningScene()
  : Node("nvblox_esdf_to_planning_scene")
  {
    esdf_service_name_ =
      declare_parameter<std::string>("esdf_service_name", "/nvblox_node/get_esdf_and_gradient");
    planning_scene_topic_ = declare_parameter<std::string>("planning_scene_topic", "/planning_scene");
    frame_id_ = declare_parameter<std::string>("frame_id", "world_manipulator");
    publish_rate_ = declare_parameter<double>("publish_rate", 2.0);
    occupied_distance_ = declare_parameter<double>("occupied_distance", 0.0);
    octomap_resolution_ = declare_parameter<double>("octomap_resolution", 0.03);
    update_esdf_ = declare_parameter<bool>("update_esdf", true);
    visualize_esdf_ = declare_parameter<bool>("visualize_esdf", false);
    max_occupied_voxels_ = declare_parameter<int>("max_occupied_voxels", 50000);
    aabb_min_m_ = declareVectorParameter(this, "aabb_min_m", {-1.0, -1.0, -0.1});
    aabb_size_m_ = declareVectorParameter(this, "aabb_size_m", {2.0, 2.0, 1.5});

    esdf_client_ = create_client<nvblox_msgs::srv::EsdfAndGradients>(esdf_service_name_);
    planning_scene_pub_ = create_publisher<moveit_msgs::msg::PlanningScene>(
      planning_scene_topic_, rclcpp::QoS(1).reliable());

    const auto period = std::chrono::duration<double>(1.0 / std::max(0.1, publish_rate_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&NvbloxEsdfToPlanningScene::requestEsdf, this));

    RCLCPP_INFO(
      get_logger(),
      "Publishing nvblox ESDF as PlanningScene OctoMap "
      "(service=%s, topic=%s, frame=%s, occupied_distance=%.3f, octomap_resolution=%.3f)",
      esdf_service_name_.c_str(), planning_scene_topic_.c_str(), frame_id_.c_str(),
      occupied_distance_, octomap_resolution_);
  }

private:
  void requestEsdf()
  {
    if (request_pending_) {
      return;
    }

    if (!esdf_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "ESDF service '%s' is not ready", esdf_service_name_.c_str());
      return;
    }

    auto request = std::make_shared<nvblox_msgs::srv::EsdfAndGradients::Request>();
    request->update_esdf = update_esdf_;
    request->visualize_esdf = visualize_esdf_;
    request->use_aabb = true;
    request->frame_id = frame_id_;
    request->aabb_min_m.x = aabb_min_m_[0];
    request->aabb_min_m.y = aabb_min_m_[1];
    request->aabb_min_m.z = aabb_min_m_[2];
    request->aabb_size_m.x = aabb_size_m_[0];
    request->aabb_size_m.y = aabb_size_m_[1];
    request->aabb_size_m.z = aabb_size_m_[2];

    request_pending_ = true;
    esdf_client_->async_send_request(
      request,
      std::bind(&NvbloxEsdfToPlanningScene::handleEsdfResponse, this, std::placeholders::_1));
  }

  void handleEsdfResponse(
    rclcpp::Client<nvblox_msgs::srv::EsdfAndGradients>::SharedFuture future)
  {
    request_pending_ = false;
    const auto response = future.get();
    if (!response || !response->success) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "ESDF service returned an invalid response; skipping PlanningScene update");
      return;
    }

    publishPlanningScene(*response);
  }

  void publishPlanningScene(const nvblox_msgs::srv::EsdfAndGradients::Response & response)
  {
    const auto & grid = response.esdf_and_gradients;
    if (grid.layout.dim.size() < 3 || grid.data.empty() || response.voxel_size_m <= 0.0F) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "ESDF response does not contain a valid 3D grid");
      return;
    }

    const int nx = static_cast<int>(grid.layout.dim[0].size);
    const int ny = static_cast<int>(grid.layout.dim[1].size);
    const int nz = static_cast<int>(grid.layout.dim[2].size);
    if (nx <= 0 || ny <= 0 || nz <= 0) {
      return;
    }

    const double octomap_resolution = std::max(
      static_cast<double>(response.voxel_size_m), octomap_resolution_);
    const int sample_stride = std::max(
      1, static_cast<int>(std::ceil(octomap_resolution / response.voxel_size_m)));
    octomap::OcTree tree(octomap_resolution);
    int occupied_count = 0;
    bool hit_limit = false;

    for (int ix = 0; ix < nx && !hit_limit; ix += sample_stride) {
      for (int iy = 0; iy < ny && !hit_limit; iy += sample_stride) {
        for (int iz = 0; iz < nz; iz += sample_stride) {
          const int index = ix * grid.layout.dim[1].stride + iy * grid.layout.dim[2].stride + iz;
          if (index < 0 || static_cast<std::size_t>(index) >= grid.data.size()) {
            continue;
          }

          const float distance = grid.data[index];
          const bool unknown = std::fabs(distance - kNvbloxUnobservedValue) < 1.0e-3F;
          if (unknown || !std::isfinite(distance) || distance > occupied_distance_) {
            continue;
          }

          const double x = response.origin_m.x + (static_cast<double>(ix) + 0.5) * response.voxel_size_m;
          const double y = response.origin_m.y + (static_cast<double>(iy) + 0.5) * response.voxel_size_m;
          const double z = response.origin_m.z + (static_cast<double>(iz) + 0.5) * response.voxel_size_m;
          tree.updateNode(octomap::point3d(x, y, z), true);

          ++occupied_count;
          if (occupied_count >= max_occupied_voxels_) {
            hit_limit = true;
            break;
          }
        }
      }
    }

    tree.updateInnerOccupancy();

    moveit_msgs::msg::PlanningScene scene;
    scene.is_diff = true;
    scene.world.octomap.header.frame_id = frame_id_;
    scene.world.octomap.header.stamp = now();
    scene.world.octomap.origin.orientation.w = 1.0;
    if (!octomap_msgs::binaryMapToMsg(tree, scene.world.octomap.octomap)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Failed to convert ESDF occupied voxels to OctoMap message");
      return;
    }

    planning_scene_pub_->publish(scene);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "Published PlanningScene OctoMap from nvblox ESDF: occupied_voxels=%d%s, "
      "resolution=%.3f, stride=%d",
      occupied_count, hit_limit ? " (limited)" : "", octomap_resolution, sample_stride);
  }

  std::string esdf_service_name_;
  std::string planning_scene_topic_;
  std::string frame_id_;
  double publish_rate_ = 2.0;
  double occupied_distance_ = 0.0;
  double octomap_resolution_ = 0.03;
  bool update_esdf_ = true;
  bool visualize_esdf_ = false;
  int max_occupied_voxels_ = 50000;
  std::vector<double> aabb_min_m_;
  std::vector<double> aabb_size_m_;
  bool request_pending_ = false;

  rclcpp::Client<nvblox_msgs::srv::EsdfAndGradients>::SharedPtr esdf_client_;
  rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NvbloxEsdfToPlanningScene>());
  rclcpp::shutdown();
  return 0;
}
