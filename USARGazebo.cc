#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include "boost_server_framework.hh"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <pthread.h>

#define _DEBUG_MSG_
#ifdef _DEBUG_MSG_
#define SCK_TXT(S,T) write(S, T, strlen(T))
#endif

//#######################################################################
//  Command Server
//#######################################################################

enum USAR_COMMAND
{
   UC_NOOP    = 0,
   UC_INIT    = 1,
   UC_SET     = 2,
   UC_GETCONF = 3,
   UC_GETGEO  = 4,
   UC_DRIVE   = 5,
   UC_GETSTARTPOSES = 6
};

struct RD
{
  char  Request;
  char  Done;
  void  Init(void) { }
        RD(void) : Request(0), Done(0) {Init();}
};

//#######################################################################
//  Command Server Class Define
//////////////////////////////////////////////////////////////////
// USARcommand 
struct USARcommand
{
  //////////////////////////////////////////////////////////////////
  // USARcommand.Variables
  Server_Framework<USARcommand>&_parent;
  boost::asio::io_service       _ioservice;
  boost::asio::ip::tcp::socket  _socket;
  boost::asio::streambuf        _buffer;
  boost::thread                 _thread;
  // Add your own variables here
  RD   Msg, Spawn;
  int  Spawned;
  char model_name[100], own_name[100], topic_root[100];
  gazebo::math::Vector3 spawn_location;
  gazebo::math::Quaternion spawn_direction;

  //////////////////////////////////////////////////////////////////
  // USARcommand.Constructor
  void Init(void) { }
  USARcommand(Server_Framework<USARcommand>&parent) : 
    _parent(parent), _socket(_ioservice), Spawned(0), Msg(), Spawn() {Init();}

  //////////////////////////////////////////////////////////////////
  // USARcommand.Child_Session_Loop_Core
  void Child_Session_Loop_Core(void)
  {
    boost::system::error_code err;
//std::cout << "Child_Session_Loop a [" << this << "]" << std::endl;
    boost::asio::read_until(_socket, _buffer , "\r\n", err);
//std::cout << "Child_Session_Loop b [" << this << "]" << std::endl;
/* SAMPLE CODE : Display received data for debug
    std::iostream st(&_buffer);
    std::stringstream s;
    s << st.rdbuf();
    std::cout << CString(s.str().c_str()) << std::endl;
    st << s.str() << std::endl;
*/
/* SAMPLE CODE : Sendback received data for debug
    boost::asio::write(_socket, _buffer);
*/
//std::cout << "Child_Session_Loop c [" << this << "]" << std::endl;
//std::cout << "Child_Session_Loop d [" << this << "]" << std::endl;
  }

  //////////////////////////////////////////////////////////////////
  // USARcommand.Accept_Process
  void Accept_Process(void)
  {
//std::cout << "Accept_Process a [" << this << "]" << std::endl;
/* SAMPLE CODE : Sendback Accepted Acknowledgment for debug
    boost::asio::streambuf  ack_comment;
    std::iostream st(&ack_comment);
    st << "+ -- Accepted ["  << this << "]" << std::endl;;
    boost::asio::write(_socket, ack_comment);
*/
//std::cout << "Accept_Process b [" << this << "]" << std::endl;
    while(1)
    {
      switch(check_command_from_USARclient())
      {
      case UC_INIT : UC_INIT_spawn_a_robot();
               break;
      case UC_GETSTARTPOSES : UC_GETSTARTPOSES_give_start_poses();
               break;
      }
//      Child_Session_Loop_Core();
    }
//std::cout << "Accept_Process c [" << this << "]" << std::endl;
  }
  //////////////////////////////////////////////////////////////////
  // USARcommand.functions
  int check_command_from_USARclient(void);
  void UC_INIT_spawn_a_robot(void);
  void UC_INIT_record_spawn_param(char* own_name, char* model_name
                   , float x, float y, float z, float q1, float q2, float q3);
  void UC_GETSTARTPOSES_give_start_poses(void);
};

//#######################################################################
//  USAR Command fetch
int USARcommand::check_command_from_USARclient(void)
{
//  1.read 1 line from _socket
//  2.recognize a command
//  3.read parameters and set it into member variables
//  4.if need, set or reset flag
  int nread;
  boost::system::error_code err;
  nread = boost::asio::read_until(_socket, _buffer , "\r\n", err);
//std::cout << "Child_Session_Loop a [" << this << "]" << std::endl;
/* error routine : anyone write this with boost....
  if(nread < 0) {
	perror("Could not read socket 3000");
    boost::asio::close(_socket); 
	pthread_detach(pthread_self());
  } else if(nread == 0) {
	perror("EOF, should break connection");
    boost::asio::close(_socket);
// could not perform rbuf_num--, because there can be robots with a higher number
	pthread_detach(pthread_self());
  }
*/
// DEBUG information output  
  std::iostream st(&_buffer);
  std::stringstream s;
  s << st.rdbuf();
  printf("COMMAND = %s\n", s.str().c_str() );
  if(0 == strncmp(s.str().c_str(),"INIT",4))
  {
    if(0 == Spawned)
    {
      UC_INIT_record_spawn_param((char*)"Robo_A"
        , (char*)"pioneer3at_with_sensors", 1, -2, 0, 0, 0, 0);
      return UC_INIT;
    }
  }
  else if(0 == strncmp(s.str().c_str(),"GETSTARTPOSES",13))
  {
//    perror("GETSTARTPOSES, should give spawn_location");
    return UC_GETSTARTPOSES;
  }
  return UC_NOOP;
}

//#######################################################################
//  USARcommand.UC_INIT_spawn_a_robot
void USARcommand::UC_INIT_spawn_a_robot(void)
{
  char	model_cmd[100];
    // Already a robot has been spawned, then return
  if(0 != Spawned)
    return;
    // Create a new transport node
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
    // Initialize the node with the world name
  node->Init();
    // Create a publisher on the ~/factory topic
  gazebo::transport::PublisherPtr factoryPub
    = node->Advertise<gazebo::msgs::Factory>("~/factory");
    // Create the message
  gazebo::msgs::Factory msg;
    // Prepare command
  sprintf(model_cmd, "model://%s", model_name);
    // Model file to load
  msg.set_sdf_filename(model_cmd);
   // msg.set_edit_name(own_name);
    // Pose to initialize the model to
  gazebo::msgs::Set(msg.mutable_pose()
    , gazebo::math::Pose(spawn_location, spawn_direction));
    // Send the message
  factoryPub->Publish(msg);
    // Set Spawned flag
  Spawned = 1;
}

/* MEMO AREA
    rbuf[rbuf_num-1].Msg.Request = 0;
    rbuf[rbuf_num-1].Spawn.Done  = 1;
    if(0 == rp->Spawn.Request)
    {
      rp->Spawn.Request = 1;
      rp->Msg.Request   = 1;
    }
    if(1==rbuf[rbuf_num-1].Spawn.Request)
   MEMO AREA */

//#######################################################################
//  USARcommand.UC_INIT_record_spawn_param
void USARcommand::UC_INIT_record_spawn_param(char* _own_name, char* _model_name
                   , float x, float y, float z, float q1, float q2, float q3)
{
  gazebo::math::Vector3 _location(x, y, z);
  gazebo::math::Quaternion _direction(q1, q2, q3);
  sprintf(topic_root, "~/%s", own_name);
  strcpy(model_name, _model_name);
  strcpy(own_name, _own_name);
  spawn_location = _location;
  spawn_direction = _direction;
}

//#######################################################################
//  GETSTARTPOSES Command
//   Defined on USARSim Manual P.43
void USARcommand::UC_GETSTARTPOSES_give_start_poses(void)
{
  int nwritten;
  boost::asio::streambuf response;
  std::iostream st_response(&response);
  st_response << "NFO {StartPoses 1}{PlayerStart %g, %g, %g" << ", " << spawn_location[0] << ", " << spawn_location[1] << ", " << spawn_location[2] << " 0,0,0}";
  nwritten = boost::asio::write(_socket, _buffer);
  std::stringstream s;
  s << st_response.rdbuf();
  if(nwritten <= 0) {
    perror("NFO message could not be written, check errno");
  } else if (nwritten != strlen(s.str().c_str())) {
    perror("not whole NFO message written");
  }
  // should also convert the Quaternion spawn_direction
  // Could there be multiple StartPoses?
  // Could StartPoses be defined in a Gazebo map?
  // I know an answer of last 2 questions by adding a world plugin. M.Shimizu
/*
 char response[128]; // should be enough
 int  nwritten, bytes;
 //  perror("we should send the string NFO {StartPoses #} {Name1 x,y,z a,b,c} over the socket");
  sprintf(response, "NFO {StartPoses 1}{PlayerStart %g, %g, %g 0,0,0}", spawn_location[0], spawn_location[1], spawn_location[2]);
*/
}

//#######################################################################
//  Image Server
//#######################################################################

//#######################################################################
// SaveAsPPM saves an image for debug
// 

void  SaveAsPPM(char* filename, ConstImageStampedPtr &_msg)
{
  unsigned char* ip = (unsigned char*)_msg->image().data().c_str();
  FILE* fp = fopen(filename, "wt");
  fprintf(fp, "P3\n%d %d\n255\n", _msg->image().width(),_msg->image().height());
  for(int i=0;i<_msg->image().width()*_msg->image().height();i++)
  fprintf(fp, "%3d %3d %3d\n", ip[i*3], ip[i*3+1], ip[i*3+2]);
  fclose(fp);
}

//#######################################################################
// A structure for transferring image data
// 

//////////////////////////////////////////////////////////////////
// USARimage
struct USARimage
{
  //////////////////////////////////////////////////////////////////
  // USARimage.Variables
  Server_Framework<USARimage>  &_parent;
  boost::asio::io_service       _ioservice;
  boost::asio::ip::tcp::socket  _socket;
  boost::asio::streambuf        _buffer;
  boost::thread                 _thread;
  // Add your own variables here
  int  flag_OK, flag_U;
  char model_name[100], own_name[100], topic_root[100];
  void send_full_size_image(void);
  void send_rectangle_area_image(void);
  void imageserver_callback(ConstImageStampedPtr& _msg);

  //////////////////////////////////////////////////////////////////
  // USARimage.Constructor
  void Init(void) { }
  USARimage(Server_Framework<USARimage>&parent): 
      _parent(parent), _socket(_ioservice), flag_OK(0), flag_U(0) {Init();}

  //////////////////////////////////////////////////////////////////
  // USARimage.Child_Session_Loop_Core
  void Child_Session_Loop_Core(void)
  {
    boost::system::error_code err;
//std::cout << "Child_Session_Loop a [" << this << "]" << std::endl;
    boost::asio::read_until(_socket, _buffer , "\r\n", err);
//std::cout << "Child_Session_Loop b [" << this << "]" << std::endl;
/* SAMPLE CODE : Display received data for debug
    std::iostream st(&_buffer);
    std::stringstream s;
    s << st.rdbuf();
    std::cout << CString(s.str().c_str()) << std::endl;
    st << s.str() << std::endl;
*/
// OK or U command should be checked here`
// SAMPLE CODE : Sendback received data for debug
    boost::asio::write(_socket, _buffer);
//
//std::cout << "Child_Session_Loop c [" << this << "]" << std::endl;
//std::cout << "Child_Session_Loop d [" << this << "]" << std::endl;
  }

  //////////////////////////////////////////////////////////////////
  // USARimage.Accept_Process
  void Accept_Process(void)
  {
//std::cout << "Accept_Process a [" << this << "]" << std::endl;
// SAMPLE CODE : Sendback Accepted Acknowledgment for debug
    boost::asio::streambuf  ack_comment;
    std::iostream st(&ack_comment);
    st << "+ -- Accepted ["  << this << "]" << std::endl;;
    boost::asio::write(_socket, ack_comment);

  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();
  gazebo::transport::SubscriberPtr sub 
    = node->Subscribe("~/pioneer3at_with_sensors/chassis/r_camera/image"
      , &USARimage::imageserver_callback, this);

//std::cout << "Accept_Process b [" << this << "]" << std::endl;
    while(1)
      Child_Session_Loop_Core();
//std::cout << "Accept_Process c [" << this << "]" << std::endl;
  }
};

//#######################################################################
// Function is called everytime a message is received on topics
// 

void USARimage::imageserver_callback(ConstImageStampedPtr& _msg)
{
  if(flag_OK)
    send_full_size_image();
  else
    send_rectangle_area_image();
/* For checking to treate an image data
  static int filenumber=0;
  char filename[100];
  if(filenumber>10)
  return;
  sprintf(filename, "./tmp%02d.PPM", (filenumber++)%10);
  SaveAsPPM(filename, _msg);
*/
}

//#######################################################################
// Send camera image
// 

void USARimage::send_full_size_image(void)
{
  flag_OK = 0;
}

void USARimage::send_rectangle_area_image(void)
{
  flag_U = 0;
}

//#######################################################################
//  World Plugin
//#######################################################################

namespace gazebo
{
  class USARGazebo : public WorldPlugin
  {
  /////////////////////////////////////////////
  /// \brief Destructor
  public: virtual ~USARGazebo()
  {
//    this->connections.clear();
  }

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
/*
    this->factoryPub = this->node->Advertise<msgs::Factory>("~/factory");
    this->usarsimSub = this->node->Subscribe("~/usarsim",
                       &USARGazebo::OnUsarSim, this);
    this->connections.push_back(event::Events::ConnectPreRender(
                   boost::bind(&USARGazebo::Update, this)));
*/
  }

  pthread_t thread_hnd, imageserver_thread_hnd;
  /////////////////////////////////////////////
  // \brief Called once after Load
  private: void Init()
  {
    UCp = new Server_Framework<USARcommand>(3000);
    UIp = new Server_Framework<USARimage>(5003);
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
  /// \brief All the event connections.
  private: std::vector<event::ConnectionPtr> connections;
  public: Server_Framework<USARcommand> *UCp;
  public: Server_Framework<USARimage>   *UIp;
  };
  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(USARGazebo)
}
