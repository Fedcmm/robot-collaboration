#include "join_plugin/join_plugin.hpp"


#define PRINT(msg) std::cout << msg << std::endl;

namespace gazebo
{
    GZ_REGISTER_MODEL_PLUGIN(JoinPlugin)

    void JoinPlugin::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
    {
        std::cout << "============================ POLYGON_PLUGIN LOADED ON MODEL: " << _model->GetName() << " ============================" << std::endl;
        
        world = _model->GetWorld();
        pippo = _model;

        this->node = gazebo::transport::NodePtr(new transport::Node());
        this->node->Init(world->Name());

        this->sub = this->node->Subscribe("/polygon/join", &JoinPlugin::OnMsg, this);
        if (!this->sub)
        {
            std::cerr << "SUBSCRIPTION FALLED" << std::endl;
            return;
        }
    }

    void JoinPlugin::OnMsg(ConstGzStringPtr &_msg)
    {
        std::cout << "============================ Received: " << _msg->data() << " ============================" << std::endl;
        
        if (_msg->data() == "attach") {
            try {
                // physics::ModelPtr pippo = world->ModelByName("pippo");
                physics::ModelPtr box = world->ModelByName("box");

                if (!pippo)
                {
                    std::cerr << "NOT ANY PIPPOS" << std::endl;
                    return;
                }

                if (!box)
                {
                    std::cerr << "NOT ANY BOXXSXS" << std::endl;
                    return;
                }

                physics::LinkPtr platformLink = pippo->GetLink("pippo/platform_link");
                physics::LinkPtr boxLink = box->GetLink("box");

                physics::JointPtr joint = pippo->CreateJoint("box_joint", "fixed", platformLink, boxLink);
                if (!joint)
                {
                    std::cerr << "NOT ANY JOINTES" << std::endl;
                    return;
                }

                std::cout << "ATTACHKHCED THE CHILD: " << joint->GetChild()->GetName() << std::endl;
                std::cout << "TO THE PRARTENT: " << joint->GetParent()->GetName() << std::endl;
                std::cout << "IS CONNECTIONS: " << joint->AreConnected(platformLink, boxLink) << std::endl;
            }
            catch (common::Exception& ex) {
                std::cout << ex.GetErrorStr() << std::endl;
            }
        } else if (_msg->data() == "detach") {
            try {
                auto removed = pippo->RemoveJoint("box_joint");
                std::cout << "REMOUVOD? " << removed << std::endl;
            }
            catch (common::Exception& ex) {
                std::cout << ex.GetErrorStr() << std::endl;
            }
        }
    }
    
    /*class JoinPlugin : public WorldPlugin
    {
    public:
        void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf)
        {
            printf("=============================== POLYGON_PLUGIN LOADED ON WORLD: %s ===============================", _world->Name().c_str());
            world = _world;

            // Inizializza il nodo di trasporto
            this->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
            this->node->Init();

            // Crea un subscriber
            this->sub = this->node->Subscribe("~/polygon/join", &JoinPlugin::OnMsg, this);
        }

        void OnMsg(ConstGzStringPtr &_msg)
        {
            // Verifica il messaggio di comando
            if (_msg->data() == "join") {
                try {
                    // gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");

                    gazebo::physics::ModelPtr pippo = world->ModelByName("pippo");
                    gazebo::physics::ModelPtr box = world->ModelByName("box");

                    gazebo::physics::JointPtr joint = world->Physics()->CreateJoint("fixed", pippo);
                    joint->SetParent(pippo->GetLink("pippo/platform_link"));
                    joint->AddChild(box->GetLink("box"));

                    printf("Joint Created!!");
                }
                catch (gazebo::common::Exception& ex) {
                    printf("%s", ex.GetErrorStr().c_str());
                }
            }
        }

    protected:
        gazebo::physics::WorldPtr world;

        gazebo::transport::NodePtr node;
        gazebo::transport::SubscriberPtr sub;
    };

    GZ_REGISTER_WORLD_PLUGIN(JoinPlugin)*/
}  // namespace gazebo
