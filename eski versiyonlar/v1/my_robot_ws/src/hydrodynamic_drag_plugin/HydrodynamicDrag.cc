// YEDEK - VRX kullanıyorsan buna ihtiyaç yok
// Ama kendi custom drag istersen kullanabilirsin

#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/Vector3.hh>

namespace gz
{
namespace sim
{
namespace systems
{

class HydrodynamicDragBackup : public System,
                               public ISystemConfigure,
                               public ISystemPreUpdate
{
  public: void Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &) override
  {
    this->model = Model(_entity);

    if (!this->model.Valid(_ecm))
    {
      gzerr << "HydrodynamicDrag: Model invalid!" << std::endl;
      return;
    }

    // ---- LINK ----
    std::string linkName = "base_link";
    if (_sdf->HasElement("link_name"))
      linkName = _sdf->Get<std::string>("link_name");

    this->linkEntity = this->model.LinkByName(_ecm, linkName);
    if (!this->linkEntity)
    {
      gzerr << "HydrodynamicDrag: Link not found!" << std::endl;
      return;
    }

    // ---- DRAG ----
    if (_sdf->HasElement("linear_drag"))
      this->linearDragCoeff = _sdf->Get<double>("linear_drag");

    if (_sdf->HasElement("angular_drag"))
      this->angularDragCoeff = _sdf->Get<double>("angular_drag");

    if (_sdf->HasElement("water_level"))
      this->waterLevel = _sdf->Get<double>("water_level");

    this->linearDragX = _sdf->Get<double>("linear_drag_x", this->linearDragCoeff).first;
    this->linearDragY = _sdf->Get<double>("linear_drag_y", this->linearDragCoeff).first;
    this->linearDragZ = _sdf->Get<double>("linear_drag_z", this->linearDragCoeff).first;

    // ---- STABILITY (METACENTRIC) ----
    if (_sdf->HasElement("roll_stiffness"))
      this->rollStiffness = _sdf->Get<double>("roll_stiffness");

    if (_sdf->HasElement("pitch_stiffness"))
      this->pitchStiffness = _sdf->Get<double>("pitch_stiffness");

    gzmsg << "[HydrodynamicDragBackup] Loaded for link: " << linkName << std::endl;
  }

  public: void PreUpdate(const UpdateInfo &,
                         EntityComponentManager &_ecm) override
  {
    if (!this->linkEntity)
      return;

    auto linearVel  = _ecm.Component<components::LinearVelocity>(this->linkEntity);
    auto angularVel = _ecm.Component<components::AngularVelocity>(this->linkEntity);
    auto pose       = _ecm.Component<components::Pose>(this->linkEntity);

    if (!linearVel || !angularVel || !pose)
      return;

    // Su üstündeyse drag + stabilite yok
    if (pose->Data().Pos().Z() > this->waterLevel)
      return;

    // ---- LINEAR DRAG ----
    auto v = linearVel->Data();
    gz::math::Vector3d dragForce(
      -this->linearDragX * v.X() * std::abs(v.X()),
      -this->linearDragY * v.Y() * std::abs(v.Y()),
      -this->linearDragZ * v.Z() * std::abs(v.Z())
    );

    // ---- ANGULAR DRAG ----
    auto w = angularVel->Data();
    gz::math::Vector3d dragTorque(
      -this->angularDragCoeff * w.X() * std::abs(w.X()),
      -this->angularDragCoeff * w.Y() * std::abs(w.Y()),
      -this->angularDragCoeff * w.Z() * std::abs(w.Z())
    );

    // ---- METACENTRIC STABILITY (MEYİL DÜZELTME) ----
    auto rot = pose->Data().Rot();
    double roll  = rot.Roll();
    double pitch = rot.Pitch();

    gz::math::Vector3d stabilityTorque(
      -this->rollStiffness  * roll,
      -this->pitchStiffness * pitch,
      0.0
    );

    dragTorque += stabilityTorque;

    // ---- APPLY ----
    Link link(this->linkEntity);
    link.AddWorldWrench(_ecm, dragForce, dragTorque);
  }

  // ---- MEMBERS ----
  private: Model model{kNullEntity};
  private: Entity linkEntity{kNullEntity};

  private: double linearDragCoeff{32.0};
  private: double angularDragCoeff{22.0};

  private: double linearDragX{32.0};
  private: double linearDragY{32.0};
  private: double linearDragZ{32.0};

  private: double rollStiffness{120.0};
  private: double pitchStiffness{150.0};

  private: double waterLevel{0.0};
};

}
}
}

// ---- REGISTER ----
GZ_ADD_PLUGIN(
  gz::sim::systems::HydrodynamicDragBackup,
  gz::sim::System,
  gz::sim::systems::HydrodynamicDragBackup::ISystemConfigure,
  gz::sim::systems::HydrodynamicDragBackup::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  gz::sim::systems::HydrodynamicDragBackup,
  "gz::sim::systems::HydrodynamicDragBackup")