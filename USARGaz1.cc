#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

namespace gazebo
{
class Factory : public WorldPlugin
{
/*
  public: void Option4(physics::WorldPtr _parent)
  {
    char modelname[100];
    int modelcount=0;
    float dx, dy, dz, cx = 0, cy = 0;
    for(dz = 0; dz < 4; dz += 1)
      for(dx = 3; dx > -3; dx -= 1)
        for(dy = 3; dy > -3; dy -= 1)
        {
          sprintf(modelname,"box%d", modelcount++);
          Spawn_Box(_parent, modelname, 1.0, .03, .3, .7, cx+dx, cy+dy, 1.5+dz*1.2, modelcount/2, modelcount/3, modelcount/5);
        }
  }
*/

  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    // Option 1: Insert model from file via function call.
    // The filename must be in the GAZEBO_MODEL_PATH environment variable.

    // Option 3: Insert model from file via message passing.
    {
      // Create a new transport node
      transport::NodePtr node(new transport::Node());

      // Initialize the node with the world name
      node->Init(_parent->GetName());

      // Create a publisher on the ~/factory topic
      transport::PublisherPtr factoryPub =
      node->Advertise<msgs::Factory>("~/factory");

      // Create the message
      msgs::Factory msg;

      // Model file to load
    //  msg.set_sdf_filename("model://cylinder");
      msg.set_sdf_filename("model://pr2");

      // Pose to initialize the model to
      msgs::Set(msg.mutable_pose(),
      math::Pose(math::Vector3(1, -2, 0), math::Quaternion(0, 0, 0)));

      // Send the message
      factoryPub->Publish(msg);
    }
//    Option4(_parent);
  }
};

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(Factory)
}
