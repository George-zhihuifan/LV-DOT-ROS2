#include <gz/plugin/Register.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>

#include <depth_eval_msgs/msg/agent_pose_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace depth_eval
{

class PedestrianPoseSyncSystem:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
  public: void Configure(
      const gz::sim::Entity &,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &,
      gz::sim::EventManager &) override
  {
    if (_sdf->HasElement("pose_topic"))
    {
      this->poseTopic = _sdf->Get<std::string>("pose_topic");
    }

    if (_sdf->HasElement("model_name"))
    {
      auto elem = _sdf->FindElement("model_name");
      while (elem)
      {
        this->modelNames.push_back(elem->Get<std::string>());
        elem = elem->GetNextElement("model_name");
      }
    }

    if (!rclcpp::ok())
    {
      int argc = 0;
      char ** argv = nullptr;
      rclcpp::init(argc, argv);
    }

    this->node = std::make_shared<rclcpp::Node>("pedestrian_pose_sync_system");
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    this->subscription = this->node->create_subscription<depth_eval_msgs::msg::AgentPoseArray>(
      this->poseTopic,
      qos,
      [this](const depth_eval_msgs::msg::AgentPoseArray::SharedPtr _msg)
      {
        std::lock_guard<std::mutex> lock(this->poseMutex);
        this->latestAgents = _msg->agents;
        this->hasPoseUpdate = true;
      });

    this->executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    this->executor->add_node(this->node);
    this->spinThread = std::thread([this]()
    {
      this->executor->spin();
    });

    RCLCPP_INFO(
      this->node->get_logger(),
      "Configured pedestrian pose sync plugin on topic [%s] for %zu models.",
      this->poseTopic.c_str(),
      this->modelNames.size());
  }

  public: void PreUpdate(
      const gz::sim::UpdateInfo &,
      gz::sim::EntityComponentManager &_ecm) override
  {
    this->ResolveEntities(_ecm);

    std::vector<depth_eval_msgs::msg::AgentPose> agents;
    {
      std::lock_guard<std::mutex> lock(this->poseMutex);
      if (!this->hasPoseUpdate)
      {
        return;
      }
      agents = this->latestAgents;
      this->hasPoseUpdate = false;
    }

    for (const auto & agent : agents)
    {
      const auto & name = agent.name;
      const auto entityIt = this->modelEntities.find(name);
      if (entityIt == this->modelEntities.end() ||
          entityIt->second == gz::sim::kNullEntity)
      {
        if (this->warnedMissingModels.insert(name).second)
        {
          RCLCPP_WARN(
            this->node->get_logger(),
            "Skipping pose update for unresolved pedestrian model [%s].",
            name.c_str());
        }
        continue;
      }

      const auto & pose = agent.pose;
      gz::math::Quaterniond rotation(
        pose.orientation.w,
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z);
      rotation.Normalize();

      const gz::math::Pose3d worldPose(
        gz::math::Vector3d(pose.position.x, pose.position.y, pose.position.z),
        rotation);

      gz::sim::Model(entityIt->second).SetWorldPoseCmd(_ecm, worldPose);
    }
  }

  public: ~PedestrianPoseSyncSystem() override
  {
    if (this->executor)
    {
      this->executor->cancel();
    }
    if (this->node && this->executor)
    {
      this->executor->remove_node(this->node);
      this->node.reset();
    }
    if (this->spinThread.joinable())
    {
      this->spinThread.join();
    }
    this->executor.reset();
  }

  private: void ResolveEntities(gz::sim::EntityComponentManager &_ecm)
  {
    for (const auto & name : this->modelNames)
    {
      if (this->modelEntities.count(name) > 0 &&
          this->modelEntities[name] != gz::sim::kNullEntity)
      {
        continue;
      }

      const auto entity = _ecm.EntityByComponents(
        gz::sim::components::Name(name),
        gz::sim::components::Model());
      if (entity != gz::sim::kNullEntity)
      {
        this->modelEntities[name] = entity;
      }
    }
  }

  private: std::string poseTopic{"/pedestrian_sim/agent_states"};
  private: std::vector<std::string> modelNames;
  private: std::unordered_map<std::string, gz::sim::Entity> modelEntities;
  private: std::unordered_set<std::string> warnedMissingModels;
  private: std::mutex poseMutex;
  private: std::vector<depth_eval_msgs::msg::AgentPose> latestAgents;
  private: bool hasPoseUpdate{false};
  private: rclcpp::Node::SharedPtr node;
  private: rclcpp::Subscription<depth_eval_msgs::msg::AgentPoseArray>::SharedPtr subscription;
  private: std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor;
  private: std::thread spinThread;
};

}  // namespace depth_eval

GZ_ADD_PLUGIN(
  depth_eval::PedestrianPoseSyncSystem,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  depth_eval::PedestrianPoseSyncSystem,
  "depth_eval::PedestrianPoseSyncSystem")
