#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

/////////////////////////////////////////////////
// Example main program that publishes a message to the RoboCupRescue plugin
int main(int _argc, char **_argv)
{
  // Load gazebo
  gazebo::setupClient(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Publish to a Gazebo topic
  gazebo::transport::PublisherPtr pub =
  node->Advertise<gazebo::msgs::GzString>("~/usarsim");

  // Wait for a subscriber to connect
  pub->WaitForConnection();
  gazebo::msgs::GzString msg;
  msg.set_data(_argv[1]);
  pub->Publish(msg);

  // Make sure to shut everything down.
  gazebo::shutdown();
}
