#include <deque>
#include <sstream>
#include <string>
#include <memory>

#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/SdfEntityCreator.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/plugin/Register.hh>

#include <sdf/Root.hh>
#include <sdf/Model.hh>

#include <gz/math/Pose3.hh>
#include <gz/math/Color.hh>
#include <gz/math/Vector3.hh>

namespace trajectory_visual
{

class TrajectoryVisualPlugin
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm,
    gz::sim::EventManager & _eventMgr) override;

  void PreUpdate(
    const gz::sim::UpdateInfo & _info,
    gz::sim::EntityComponentManager & _ecm) override;

private:
  void spawnMarker(const gz::math::Vector3d & pos);

  gz::sim::Entity worldEntity_{gz::sim::kNullEntity};
  gz::sim::Entity robotEntity_{gz::sim::kNullEntity};
  std::shared_ptr<gz::sim::SdfEntityCreator> creator_;

  gz::math::Vector3d lastPos_{1e9, 0.0, 0.0};

  std::deque<gz::sim::Entity> markerEntities_;

  std::string robotName_{"my_bot"};
  double minDist_{0.15};
  double markerRadius_{0.04};
  gz::math::Color markerColor_{1.0f, 0.2f, 0.0f, 1.0f};
  std::size_t maxMarkers_{200};
  int markerCount_{0};
};

// ─── Configure ───────────────────────────────────────────────────────────────

void TrajectoryVisualPlugin::Configure(
  const gz::sim::Entity & /*_entity*/,
  const std::shared_ptr<const sdf::Element> & _sdf,
  gz::sim::EntityComponentManager & _ecm,
  gz::sim::EventManager & _eventMgr)
{
  if (_sdf->HasElement("robot_name")) {
    robotName_ = _sdf->Get<std::string>("robot_name");
  }
  if (_sdf->HasElement("min_dist")) {
    minDist_ = _sdf->Get<double>("min_dist");
  }
  if (_sdf->HasElement("marker_radius")) {
    markerRadius_ = _sdf->Get<double>("marker_radius");
  }
  if (_sdf->HasElement("color")) {
    markerColor_ = _sdf->Get<gz::math::Color>("color");
  }
  if (_sdf->HasElement("max_markers")) {
    maxMarkers_ = _sdf->Get<std::size_t>("max_markers");
  }

  _ecm.Each<gz::sim::components::World>(
    [&](const gz::sim::Entity & ent, const gz::sim::components::World *) -> bool {
      worldEntity_ = ent;
      return false;
    });

  creator_ = std::make_shared<gz::sim::SdfEntityCreator>(_ecm, _eventMgr);
}

// ─── PreUpdate ───────────────────────────────────────────────────────────────

void TrajectoryVisualPlugin::PreUpdate(
  const gz::sim::UpdateInfo & /*_info*/,
  gz::sim::EntityComponentManager & _ecm)
{
  // Find robot entity once after it's spawned
  if (robotEntity_ == gz::sim::kNullEntity) {
    _ecm.Each<gz::sim::components::Model, gz::sim::components::Name>(
      [&](const gz::sim::Entity & ent,
          const gz::sim::components::Model *,
          const gz::sim::components::Name * name) -> bool {
        if (name->Data() == robotName_) {
          robotEntity_ = ent;
          return false;
        }
        return true;
      });
    return;
  }

  const auto worldPose = gz::sim::worldPose(robotEntity_, _ecm);
  const gz::math::Vector3d pos(worldPose.Pos().X(), worldPose.Pos().Y(), 0.01);

  const double dx = pos.X() - lastPos_.X();
  const double dy = pos.Y() - lastPos_.Y();
  if (std::sqrt(dx * dx + dy * dy) >= minDist_) {
    spawnMarker(pos);
    lastPos_ = pos;
  }
}

// ─── spawnMarker ─────────────────────────────────────────────────────────────

void TrajectoryVisualPlugin::spawnMarker(const gz::math::Vector3d & pos)
{
  std::ostringstream ss;
  ss << "<sdf version='1.9'>"
     << "<model name='traj_marker_" << markerCount_++ << "'>"
     << "<static>true</static>"
     << "<pose>" << pos.X() << " " << pos.Y() << " " << pos.Z() << " 0 0 0</pose>"
     << "<link name='link'>"
     << "<visual name='vis'>"
     << "<cast_shadows>false</cast_shadows>"
     << "<geometry><sphere><radius>" << markerRadius_ << "</radius></sphere></geometry>"
     << "<material>"
     << "<ambient>" << markerColor_.R() << " " << markerColor_.G()
                    << " " << markerColor_.B() << " 1</ambient>"
     << "<diffuse>" << markerColor_.R() << " " << markerColor_.G()
                    << " " << markerColor_.B() << " 1</diffuse>"
     << "<emissive>" << markerColor_.R() * 0.4f << " " << markerColor_.G() * 0.4f
                     << " " << markerColor_.B() * 0.4f << " 1</emissive>"
     << "</material>"
     << "</visual>"
     << "</link>"
     << "</model>"
     << "</sdf>";

  sdf::Root root;
  root.LoadSdfString(ss.str());
  const sdf::Model * model = root.Model();
  if (!model) {
    return;
  }

  auto modelEnt = creator_->CreateEntities(model);
  creator_->SetParent(modelEnt, worldEntity_);
  markerEntities_.push_back(modelEnt);

  if (markerEntities_.size() > maxMarkers_) {
    creator_->RequestRemoveEntity(markerEntities_.front());
    markerEntities_.pop_front();
  }
}

}  // namespace trajectory_visual

GZ_ADD_PLUGIN(
  trajectory_visual::TrajectoryVisualPlugin,
  gz::sim::System,
  trajectory_visual::TrajectoryVisualPlugin::ISystemConfigure,
  trajectory_visual::TrajectoryVisualPlugin::ISystemPreUpdate)
