#include <polygon_base/regular_polygon.hpp>
#include <cmath>
#include <thread>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo_client.hh>


namespace polygon_plugins
{
  class Square : public polygon_base::RegularPolygon, public gazebo::WorldPlugin
  {
    public:
        void Load(gazebo::physics::WorldPtr _world, sdf::ElementPtr _sdf) override
        {
            printf("=============================== POLYGON_PLUGIN LOADED ON WORLD: %s ===============================", _world->Name().c_str());
            world = _world;

            // Inizializza il nodo di trasporto
            this->node = gazebo::transport::NodePtr(new gazebo::transport::Node());
            this->node->Init();

            // Crea un subscriber
            this->sub = this->node->Subscribe("/polygon/join", &Square::OnMsg, this);
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

        void initialize(double side_length) override
        {
            side_length_ = side_length;
        }

        double area() override
        {
            return side_length_ * side_length_;
        }

    protected:
        double side_length_;
        gazebo::physics::WorldPtr world;

        gazebo::transport::NodePtr node;
        gazebo::transport::SubscriberPtr sub;
  };

  GZ_REGISTER_WORLD_PLUGIN(Square)

  class Triangle : public polygon_base::RegularPolygon
  {
    public:
      void initialize(double side_length) override
      {
        side_length_ = side_length;
      }

      double area() override
      {
        try {
            gazebo::client::setup();
            gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");

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
        return 0.5 * side_length_ * getHeight();
      }

      double getHeight()
      {
        return sqrt((side_length_ * side_length_) - ((side_length_ / 2) * (side_length_ / 2)));
      }

    protected:
      double side_length_;
  };
}

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(polygon_plugins::Square, polygon_base::RegularPolygon)
PLUGINLIB_EXPORT_CLASS(polygon_plugins::Triangle, polygon_base::RegularPolygon)