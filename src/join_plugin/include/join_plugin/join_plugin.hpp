#ifndef JOIN_PLUGIN__JOIN_PLUGIN_HPP_
#define JOIN_PLUGIN__JOIN_PLUGIN_HPP_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{

class JoinPlugin : public ModelPlugin
{
public:
    JoinPlugin() : ModelPlugin() {}

    virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
    virtual void OnMsg(ConstGzStringPtr &_msg);

protected:
    physics::WorldPtr world;
    physics::ModelPtr robot;

    transport::NodePtr node;
    transport::SubscriberPtr sub;
};

}  // namespace gazebo

#endif  // JOIN_PLUGIN__JOIN_PLUGIN_HPP_
