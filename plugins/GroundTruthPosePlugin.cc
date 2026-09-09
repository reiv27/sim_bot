#include <chrono>
#include <memory>
#include <string>

#include <gz/msgs/pose.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>

namespace ground_truth_pose
{

class GroundTruthPosePlugin
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate
{
public:
  void Configure(
    const gz::sim::Entity &,
    const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager &,
    gz::sim::EventManager &) override
  {
    if (_sdf->HasElement("robot_name")) {
      robotName_ = _sdf->Get<std::string>("robot_name");
    }
    if (_sdf->HasElement("formation_name")) {
      formationName_ = _sdf->Get<std::string>("formation_name");
    }
    if (_sdf->HasElement("update_rate")) {
      updatePeriod_ = std::chrono::duration<double>(
        1.0 / _sdf->Get<double>("update_rate"));
    }
    robotPublisher_ = node_.Advertise<gz::msgs::Pose>("/profiling/my_bot/pose");
    formationPublisher_ = node_.Advertise<gz::msgs::Pose>("/profiling/rigid_group/pose");
  }

  void PostUpdate(
    const gz::sim::UpdateInfo & _info,
    const gz::sim::EntityComponentManager & _ecm) override
  {
    if (_info.paused || _info.simTime < nextPublishTime_) {
      return;
    }
    nextPublishTime_ = _info.simTime +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(updatePeriod_);
    publishPose(robotName_, robotEntity_, robotPublisher_, _ecm);
    publishPose(formationName_, formationEntity_, formationPublisher_, _ecm);
  }

private:
  void publishPose(
    const std::string & name,
    gz::sim::Entity & cachedEntity,
    gz::transport::Node::Publisher & publisher,
    const gz::sim::EntityComponentManager & ecm)
  {
    if (cachedEntity == gz::sim::kNullEntity) {
      ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
        [&](const gz::sim::Entity & entity,
            const gz::sim::components::Model *,
            const gz::sim::components::Name * entityName) -> bool {
          if (entityName->Data() == name) {
            cachedEntity = entity;
            return false;
          }
          return true;
        });
    }
    if (cachedEntity == gz::sim::kNullEntity) {
      return;
    }
    const auto pose = gz::sim::worldPose(cachedEntity, ecm);
    gz::msgs::Pose message;
    message.set_name(name);
    message.mutable_position()->set_x(pose.Pos().X());
    message.mutable_position()->set_y(pose.Pos().Y());
    message.mutable_position()->set_z(pose.Pos().Z());
    message.mutable_orientation()->set_x(pose.Rot().X());
    message.mutable_orientation()->set_y(pose.Rot().Y());
    message.mutable_orientation()->set_z(pose.Rot().Z());
    message.mutable_orientation()->set_w(pose.Rot().W());
    publisher.Publish(message);
  }

  gz::transport::Node node_;
  gz::transport::Node::Publisher robotPublisher_;
  gz::transport::Node::Publisher formationPublisher_;
  gz::sim::Entity robotEntity_{gz::sim::kNullEntity};
  gz::sim::Entity formationEntity_{gz::sim::kNullEntity};
  std::string robotName_{"my_bot"};
  std::string formationName_{"rigid_group"};
  std::chrono::duration<double> updatePeriod_{0.02};
  std::chrono::steady_clock::duration nextPublishTime_{0};
};

}  // namespace ground_truth_pose

GZ_ADD_PLUGIN(
  ground_truth_pose::GroundTruthPosePlugin,
  gz::sim::System,
  ground_truth_pose::GroundTruthPosePlugin::ISystemConfigure,
  ground_truth_pose::GroundTruthPosePlugin::ISystemPostUpdate)
