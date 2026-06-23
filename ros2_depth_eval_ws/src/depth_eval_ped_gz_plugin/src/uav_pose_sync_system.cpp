// Minimal Gazebo system plugin that teleports a single UAV model
// to the latest pose received on a ROS topic. Bypasses SetEntityPose service
// by writing the pose in the
// physics-thread PreUpdate via Model::SetWorldPoseCmd.

#include <gz/plugin/Register.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace depth_eval
{

class UavPoseSyncSystem :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity &,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager &,
    gz::sim::EventManager &) override
  {
    if (sdf->HasElement("pose_topic")) {
      pose_topic_ = sdf->Get<std::string>("pose_topic");
    }
    if (sdf->HasElement("model_name")) {
      model_name_ = sdf->Get<std::string>("model_name");
    }

    if (!rclcpp::ok()) {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
    }

    node_ = std::make_shared<rclcpp::Node>("uav_pose_sync_system");
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    subscription_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_, qos,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mtx_);
        latest_pose_ = msg->pose;
        has_update_ = true;
      });

    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });

    RCLCPP_INFO(
      node_->get_logger(),
      "UavPoseSyncSystem: model=[%s] topic=[%s]",
      model_name_.c_str(), pose_topic_.c_str());
  }

  void PreUpdate(
    const gz::sim::UpdateInfo &,
    gz::sim::EntityComponentManager & ecm) override
  {
    if (entity_ == gz::sim::kNullEntity) {
      entity_ = ecm.EntityByComponents(
        gz::sim::components::Name(model_name_),
        gz::sim::components::Model());
      if (entity_ == gz::sim::kNullEntity) {
        return;
      }
    }

    geometry_msgs::msg::Pose pose;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      if (!has_update_) {
        return;
      }
      pose = latest_pose_;
      has_update_ = false;
    }

    gz::math::Quaterniond q(
      pose.orientation.w, pose.orientation.x,
      pose.orientation.y, pose.orientation.z);
    q.Normalize();
    const gz::math::Pose3d world_pose(
      gz::math::Vector3d(pose.position.x, pose.position.y, pose.position.z),
      q);

    gz::sim::Model(entity_).SetWorldPoseCmd(ecm, world_pose);
  }

  ~UavPoseSyncSystem() override
  {
    if (executor_) executor_->cancel();
    if (spin_thread_.joinable()) spin_thread_.join();
    executor_.reset();
    node_.reset();
  }

private:
  std::string pose_topic_{"/uav_motion/pose_cmd"};
  std::string model_name_{"uav_main"};
  gz::sim::Entity entity_{gz::sim::kNullEntity};
  std::mutex mtx_;
  bool has_update_{false};
  geometry_msgs::msg::Pose latest_pose_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
  std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spin_thread_;
};

}  // namespace depth_eval

GZ_ADD_PLUGIN(
  depth_eval::UavPoseSyncSystem,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)
