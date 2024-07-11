#include "force_plugin/force_plugin.hpp"
#include <gz/plugin/Register.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <gz/sim/Model.hh>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>

GZ_ADD_PLUGIN(force_plugin::ForcePlugin, gz::sim::System, gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)

namespace force_plugin
{
  ForcePlugin::ForcePlugin() : forceX_(0), forceY_(0), torqueZ_(0)
  {
  }

  ForcePlugin::~ForcePlugin()
  {
  }

  void ForcePlugin::Configure(const gz::sim::Entity& entity, const std::shared_ptr<const sdf::Element>& /*sdf*/, gz::sim::EntityComponentManager& ecm, gz::sim::EventManager& /*eventMgr*/)
  {
    auto model = gz::sim::Model(entity);
    baseLinkEntity_ = model.LinkByName(ecm, "base_link");

    node_ = std::make_shared<rclcpp::Node>("force_plugin");
    sub_ = node_->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&ForcePlugin::OnTwistMsg, this, std::placeholders::_1));
  }

  void ForcePlugin::PreUpdate(const gz::sim::UpdateInfo& /*info*/, gz::sim::EntityComponentManager& ecm)
  {
    auto linVelComp = ecm.Component<gz::sim::components::LinearVelocityCmd>(baseLinkEntity_);
    auto angVelComp = ecm.Component<gz::sim::components::AngularVelocityCmd>(baseLinkEntity_);

    if (linVelComp && angVelComp)
    {
      linVelComp->Data().Set(forceX_, forceY_, 0);
      angVelComp->Data().Set(0, 0, torqueZ_);
    }
    else
    {
      ecm.CreateComponent(baseLinkEntity_, gz::sim::components::LinearVelocityCmd(gz::math::Vector3d(forceX_, forceY_, 0)));
      ecm.CreateComponent(baseLinkEntity_, gz::sim::components::AngularVelocityCmd(gz::math::Vector3d(0, 0, torqueZ_)));
    }
  }

  void ForcePlugin::OnTwistMsg(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    forceX_ = msg->linear.x;
    forceY_ = msg->linear.y;
    torqueZ_ = msg->angular.z;
  }
}
