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

struct RD
{
  char  Request;
  char  Done;
  void  Init(void) { }
  RD(void) : Request(0), Done(0) { Init(); }
};

struct UC_Param
{
  char _Name[20];
  char _Value[50];
  void Disp(void) { std::cout << _Name << " " << _Value << std::endl; }
};

#ifndef _MIN
#define _MIN(X,Y) (((X)<(Y))?(X):(Y))
#endif

struct Break_UC_Command_Into_Params
{
  int                 _Err_flag;
  UC_Param*           _ucpp;
  std::set<UC_Param*> _UC_Param_list;
  ~Break_UC_Command_Into_Params() { Remove_all(); }
  Break_UC_Command_Into_Params(void) { Init(); }
  Break_UC_Command_Into_Params(char* cb) { Init(); Break_Into(cb); }
  void Init(void) { _Err_flag = 0; _UC_Param_list.clear(); }
  void Break_Into(char* _cmdbuf)
  {
  // 1st : copy command string 
    char* cmdbuf = new char[strlen(_cmdbuf)];
    int   _in_brace = 0, _after_space = 0, _Name_len, _Value_len;
    char* _Name_Start = NULL;
    char* _Value_Start = NULL;
    UC_Param _tmp;
    for(int i = 0; cmdbuf[i] != 0; i++)
    {
      if(0 == _in_brace && '{' != cmdbuf[i])
        continue;
      else if('{' == cmdbuf[i])
      {
        _in_brace++;
	continue;
      }
      else if(0 < _in_brace && NULL == _Name_Start)
      {
        _Name_Start = &cmdbuf[i];
	continue;
      }
      else if(0 < _in_brace && isspace(cmdbuf[i]))
      {
        _after_space++;
	if(0 < _in_brace && 1 == _after_space && NULL != _Name_Start)
	{
	  _Name_len = (int)(&cmdbuf[i] - _Name_Start);
	  strncpy(_tmp._Name, _Name_Start
	            , _MIN(_Name_len, sizeof(_tmp._Name)-1));
          _tmp._Name[_MIN(_Name_len, sizeof(_tmp._Name)-1)] = 0;
	}
	continue;
      }
      else if(0 < _in_brace && 0 < _after_space && NULL == _Value_Start)
      {
        _Value_Start = &cmdbuf[i];
	continue;
      }
      else if(0 < _in_brace && '}' == cmdbuf[i])
      {
        _in_brace--;
	if(0 == _in_brace && NULL != _Value_Start)
	{
	  _Value_len = (int)(&cmdbuf[i] - _Value_Start);
	  strncpy(_tmp._Value, _Value_Start
	            , _MIN(_Value_len, sizeof(_tmp._Value)-1));
          _tmp._Value[_MIN(_Value_len, sizeof(_tmp._Value)-1)] = 0;
          Add_Param(_tmp);
	  _Name_Start = _Value_Start = NULL;
	  _after_space = 0;
	  continue;
	}
	else if(0 > _in_brace)
	{
	  _Err_flag = -1;
	  Remove_all();
	  delete cmdbuf;
	  return;
	}
	continue;
      }
/*      else
      {
        if(0 < _in_brace)
	{
	}
      }*/
    }
    if(0 != _in_brace || NULL != _Name_Start || NULL != _Value_Start)
    {
      _Err_flag = -2;
      Remove_all();
      delete cmdbuf;
      return;
    }
    Disp();
  }
  void Add_Param(UC_Param& up)
  {
    UC_Param* _newp;
    _newp = new UC_Param;
    *_newp = up;
_newp->Disp();
    _UC_Param_list.insert(_newp);
  }
  void Remove_all(void)
  {
    for(typename std::set<UC_Param*>::iterator
      i = _UC_Param_list.begin(); i != _UC_Param_list.end(); i++)
        delete *i;
    Init();
  }
  int Size(void)
  {
    return _UC_Param_list.size();
  }
  void Disp(void)
  {
    std::cout << "The number of elements : " << Size() << std::endl;
    for(typename std::set<UC_Param*>::iterator
      i = _UC_Param_list.begin(); i != _UC_Param_list.end(); i++)
    {
      UC_Param* upp = *i;
      upp->Disp();
    }
  }
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
  char*ucbuf; // a pointer of USARSim command string buffer in GameBot protocol
  gazebo::math::Vector3 spawn_location;
  gazebo::math::Quaternion spawn_direction;

  //////////////////////////////////////////////////////////////////
  // USARcommand.Constructor
  void Init(void) { }
  USARcommand(Server_Framework<USARcommand>&parent) : 
    _parent(parent), _socket(_ioservice), Spawned(0), Msg(), Spawn() 
  { Init(); }

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
      UC_check_command_from_USARclient();
//    Child_Session_Loop_Core();
    }
//std::cout << "Accept_Process c [" << this << "]" << std::endl;
  }

  //////////////////////////////////////////////////////////////////
  // Get_Topics_List
  void Get_Topics_List(void) // still under construction
  {
  /* SAMPLE CODE to get a list of topics from https://bitbucket.org/osrf/gazebo/src/5eed0402ea08b3dff261d25ef8da82db426bbab6/tools/gz_topic.cc?at=default#cl-87
    std::string data;
    msgs::Packet packet;
    msgs::Request request;
    msgs::GzString_V topics;
    transport::ConnectionPtr _connection = transport::connectToMaster();
    if(connection)
    {
      request.set_id(0);
      request.set_request("get_topics");
      _connection->EnqueueMsg(msgs::Package("request", request), true);
      _connection->Read(data);
      packet.ParseFromString(data);
      topics.ParseFromString(packet.serialized_data());
      for (int i = 0; i < topics.data_size(); ++i)
      {
        std::cout << topics.data(i) << std::endl;
      }
    }
    _connection.reset();
  */
  }

  //////////////////////////////////////////////////////////////////
  // Send_SENS
  void Send_SENS(void)
  {
    if(0 == Spawned)
      return;
      // 1. Check robot's sensors from topics list
    Get_Topics_List();
      // 2. send SENS of each sensors
      // SAMPLE CODE For debug
    // A code for Debugging
    if(1 == Spawned)
    {
      printf("robot name = %s\n", own_name);
      Spawned = 0;
    }
    //
  }

  //////////////////////////////////////////////////////////////////
  // USARcommand.functions
  void UC_check_command_from_USARclient(void);
};

//////////////////////////////////////////////////////////////////
// UC_INIT
struct UC_INIT
{
  //#######################################################################
  //  Variables
  USARcommand& _parent;
  //#######################################################################
  //  The constructor
  UC_INIT(USARcommand& parent) : _parent(parent)
  {
      // Record this robot information
    read_params_from_uc_command();
    record_robot_param((char*)"Robo_A"
      , (char*)"pioneer3at_with_sensors", 1, -2, 0, 0, 0, 0);
      // Spawn a robot
    spawn_a_robot();
  }
  //#######################################################################
  //  read_params_from_uc_command
  void read_params_from_uc_command(void)
  {
    Break_UC_Command_Into_Params BUCIP(_parent.ucbuf);
  }
  //#######################################################################
  //  spawn_a_robot
  void spawn_a_robot(void)
  {
    char	model_cmd[100];
      // Already a robot has been spawned, then return
    if(1 == _parent.Spawned)
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
    sprintf(model_cmd, "model://%s", _parent.model_name);
      // Model file to load
    msg.set_sdf_filename(model_cmd);
      // Set this robot's own name
    // msg.set_edit_name(_parent.own_name); Please fix this function! > Dr.Nate
      // Pose to initialize the model to
    gazebo::msgs::Set(msg.mutable_pose()
      , gazebo::math::Pose(_parent.spawn_location, _parent.spawn_direction));
      // Send the message
    factoryPub->Publish(msg);
      // Set Spawned flag
    _parent.Spawned = 1;
  }
  //#######################################################################
  //  record_robot_param
  void record_robot_param(char* _own_name, char* _model_name
                     , float x, float y, float z, float q1, float q2, float q3)
  {
      // Already a robot has been spawned, then return
    if(1 == _parent.Spawned)
      return;
    gazebo::math::Vector3 _location(x, y, z);
    gazebo::math::Quaternion _direction(q1, q2, q3);
    sprintf(_parent.topic_root, "~/%s", _own_name);
    strcpy(_parent.own_name, _own_name);
    strcpy(_parent.model_name, _model_name);
    _parent.spawn_location = _location;
    _parent.spawn_direction = _direction;
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
};

//////////////////////////////////////////////////////////////////
//  UC_GETSTARTPOSES
//   Defined on USARSim Manual P.43
struct UC_GETSTARTPOSES
{
  //#######################################################################
  //  Variables
  USARcommand& _parent;
  //#######################################################################
  //  The constructor
  UC_GETSTARTPOSES(USARcommand& parent) : _parent(parent)
  {
    int                    nwritten;
    boost::asio::streambuf response;
    std::iostream          st_response(&response);
    st_response << "NFO {StartPoses 1}{PlayerStart "<<_parent.spawn_location[0]
      << ", " << _parent.spawn_location[1] << ", " << _parent.spawn_location[2] 
      << " 0,0,0}" << std::endl;
//std::cout << st_response;
    nwritten = boost::asio::write(_parent._socket, response);
//    std::stringstream s;
//    s << st_response.rdbuf();
    if(nwritten <= 0) {
      perror("NFO message could not be written, check errno");
    }
    /*
    else if (nwritten != strlen(s.str().c_str())) {
      perror("not whole NFO message written");
    }
    */
    // should also convert the Quaternion spawn_direction
    // Could there be multiple StartPoses?
    // Could StartPoses be defined in a Gazebo map?
    // I know an answer of last 2 questions by adding a world plugin. M.Shimizu
  /*
   char response[128]; // should be enough
   int  nwritten, bytes;
   //  perror("we should send the string NFO {StartPoses #} {Name1 x,y,z a,b,c} over the socket");
    sprintf(response, "NFO {StartPoses 1}{PlayerStart %g, %g, %g 0,0,0}"
      , _parent.spawn_location[0], _parent.spawn_location[1]
      , _parent.spawn_location[2]);
  */
  }
};

//#######################################################################
//  USAR Command fetch
void USARcommand::UC_check_command_from_USARclient(void)
{
  int nread;
  boost::system::error_code err;
  nread = boost::asio::read_until(_socket, _buffer , "\r\n", err);
//std::cout << "Child_Session_Loop a [" << this << "]" << std::endl;
   // error routine : anyone write this with boost....
  if(nread < 0)
  {
    perror("Could not read socket 3000");
//  close(_socket);
    _parent.Remove_Child_Session(this); // from child session list
    _thread.join();
//  pthread_detach(pthread_self());
  } 
  else if(nread == 0) 
  {
    perror("EOF, should break connection");
//  close(_socket);
    _parent.Remove_Child_Session(this); // from child session list
//  pthread_detach(pthread_self());
  }
  std::iostream st(&_buffer);
  std::stringstream s;
  s << st.rdbuf();
  ucbuf = (char*)s.str().c_str();
// DEBUG information output  
printf("COMMAND = %s\n", ucbuf );
  if(0 == strncmp(ucbuf, "INIT", 4))
  {
    UC_INIT UC_init(*this);
  }
  else if(0 == strncmp(ucbuf, "GETSTARTPOSES", 13))
  {
    UC_GETSTARTPOSES UC_getstartposes(*this);
  }
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
  void send_full_size_image(ConstImageStampedPtr& _msg);
  void send_rectangle_area_image(ConstImageStampedPtr& _msg);
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
    std::iostream st(&_buffer);
    std::stringstream s;
    s << st.rdbuf();
//printf("COMMAND = %s\n", s.str().c_str() );
    if(0 == strncmp(s.str().c_str(),"OK",2))
    {
      flag_OK = 1;
    }
    else if(0 == strncmp(s.str().c_str(),"U[",2))
    {
      flag_U = 1;
    }
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
/* SAMPLE CODE : Sendback Accepted Acknowledgment for debug
    boost::asio::streambuf  ack_comment;
    std::iostream st(&ack_comment);
    st << "+ -- Accepted ["  << this << "]" << std::endl;;
    boost::asio::write(_socket, ack_comment);
*/
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
    send_full_size_image(_msg);
  else if(flag_U)
    send_rectangle_area_image( _msg);
}

//#######################################################################
// Send camera image
// 

void USARimage::send_full_size_image(ConstImageStampedPtr& _msg)
{
  flag_OK = 0;
// For checking to treate an image data
  static int filenumber=0;
  char filename[100];
  if(filenumber>10)
  return;
  sprintf(filename, "./tmp%02d.PPM", (filenumber++)%10);
  SaveAsPPM(filename, _msg);

}

void USARimage::send_rectangle_area_image(ConstImageStampedPtr& _msg)
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

  public: 
   USARGazebo(void):counter1(0){}
  virtual ~USARGazebo()
  {
    connections.clear();
  }

  ///////////////////////////////////////////////
  // \brief Load the robocup rescue plugin
  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/)
  {
    world = _parent;
    // Create a new transport node
    node.reset(new transport::Node());
    // Initialize the node with the world name
    node->Init(_parent->GetName());
    // Create a publisher on the ~/factory topic
/*
    factoryPub = node->Advertise<msgs::Factory>("~/factory");
    usarsimSub = node->Subscribe("~/usarsim",
                       &USARGazebo::OnUsarSim, this);
*/
    connections.push_back(event::Events::ConnectWorldUpdateBegin(
                       boost::bind(&USARGazebo::Send_SENS, this)));
  }

  /////////////////////////////////////////////
  /// \brief Called every PreRender event. See the Load function.
  private: void Send_SENS()
  {
  // Add codes to send SENS and other status data here
  // Because they have to be in a serial
    for(typename std::set<USARcommand*>::iterator
      i = UCp->_Child_Session_list.begin(); 
        i != UCp->_Child_Session_list.end(); i++)
    { // i is a pointer of each USARcommand
//      USARcommand* child_ucp = (USARcommand*)&**i;
      USARcommand* child_ucp = *i;
      child_ucp->Send_SENS();
    }
  }

  /////////////////////////////////////////////
  // \brief Called once after Load
  private: void Init()
  {
    UCp = new Server_Framework<USARcommand>(3000);
    UIp = new Server_Framework<USARimage>(5003);
  }

private:
  /// \brief Gazebo communication node
  transport::NodePtr node;
  /// \brief Gazebo factory publisher
  transport::PublisherPtr factoryPub;
  /// \brief Gazebo subscriber to the ~/usarsim topic.
  /// This is a stand-in for the usarsim interface
  transport::SubscriberPtr usarsimSub;
  /// \brief Keep a pointer to the world.
  physics::WorldPtr world;
  /// \brief All the event connections.
  std::vector<event::ConnectionPtr> connections;
public:
  Server_Framework<USARcommand> *UCp;
  Server_Framework<USARimage>   *UIp;
  int counter1;
  };
  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(USARGazebo)
}
