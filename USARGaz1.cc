#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

namespace gazebo
{
class USARGaz : public WorldPlugin
{
  ///////////////////////////////////////////////
  // \brief Load the robocup rescue plugin
  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    this->world = _parent;

    // Create a new transport node
    this->node.reset(new transport::Node());

    // Initialize the node with the world name
    this->node->Init(_parent->GetName());

    // Create a publisher on the ~/factory topic
    this->factoryPub = this->node->Advertise<msgs::Factory>("~/factory");

    this->usarsimSub = this->node->Subscribe("~/usarsim",
        &USARGaz::OnUsarSim, this);
  }

  ///////////////////////////////////////////////
  /// \brief Receive a command from a usarsim program
  public: void OnUsarSim(ConstGzStringPtr &_msg)
  {
    if (_msg->data() == "init")
    {
      // Create the message
      msgs::Factory msg;

      // Model file to load
      msg.set_sdf_filename("model://pr2");

      // Pose to initialize the model to
      msgs::Set(msg.mutable_pose(),
          math::Pose(math::Vector3(1, -2, 0), math::Quaternion(0, 0, 0)));

      // Send the message
      this->factoryPub->Publish(msg);
    }
    else if (_msg->data() == "pose")
    {
      this->world->SetPaused(true);
      // Get a pointer to a model
      physics::ModelPtr model = this->world->GetModel("pr2");
      if (model)
      {
        model->SetWorldPose(math::Pose(0, 0, 0.1, 0, 0, 0));
      }
      else
      {
        std::cerr << "Unable to find model with name[my_model_name]\n";
      }
      this->world->SetPaused(false);
    }
  }

  /// \brief Gazebo communication node
  private: transport::NodePtr node;

  /// \brief Gazebo factory publisher
  private: transport::PublisherPtr factoryPub;

  /// \brief Gazebo subscriber to the ~/usarsim topic.
  /// This is a stand-in for the usarsim interface
  private: transport::SubscriberPtr usarsimSub;

  /// \brief Keep a pointer to the world.
  private: physics::WorldPtr world;
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(USARGaz)
}
