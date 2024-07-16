#include "join_plugin/join_plugin.hpp"

namespace gazebo
{
    GZ_REGISTER_MODEL_PLUGIN(JoinPlugin)

    void JoinPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        std::cout << "========================= JOIN_PLUGIN LOADED ON MODEL: " << _model->GetName() << " =========================" << std::endl;
        
        world = _model->GetWorld();
        robot = _model;

        this->node = gazebo::transport::NodePtr(new transport::Node());
        this->node->Init(world->Name());

        this->sub = this->node->Subscribe("/polygon/join", &JoinPlugin::OnMsg, this);
        if (!this->sub)
        {
            std::cerr << "Subscription to join topic failed" << std::endl;
            return;
        }
    }

    void JoinPlugin::OnMsg(ConstGzStringPtr &_msg)
    {
        if (_msg->data() == "attach") {
            try {
                physics::ModelPtr box = world->ModelByName("box");

                physics::LinkPtr platformLink = robot->GetLink("robot/platform_link");
                physics::LinkPtr boxLink = box->GetLink("box");

                physics::JointPtr joint = robot->CreateJoint("box_joint", "fixed", platformLink, boxLink);
                if (!joint)
                {
                    std::cerr << "Failed to create joint" << std::endl;
                    return;
                }

                std::cout << "Attached joint: " << joint->GetChild()->GetName() << std::endl;
                std::cout << "To parent: " << joint->GetParent()->GetName() << std::endl;
            }
            catch (common::Exception& ex) {
                std::cout << ex.GetErrorStr() << std::endl;
            }
        } else if (_msg->data() == "detach") {
            try {
                auto removed = robot->RemoveJoint("box_joint");
                if (removed) {
                    std::cout << "Joint removed" << std::endl;
                }
            }
            catch (common::Exception& ex) {
                std::cout << ex.GetErrorStr() << std::endl;
            }
        }
    }
}  // namespace gazebo
