// Desctiption: 
// This program's a translator between USARSim protocol and Gazebo protocol.
// This file is a primaly file of this program. 
// You can add some codes for treating USARsim command which you need in this file by copying and editing alread existing other codes of USARSim protcol.
// UC_INIT class is a good sample code for adding a new code for treating a new USARSim command.
//
// Author : Prof.Arnoud Visser, Dr.Nate Koenig, Masaru Shimizu
// E-Mail : shimizu@sist.chukyo-u.ac.jp
// Date   : 3.2015-5.2015
//

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/math/gzmath.hh>
#include "boost_server_framework.hh"
#include "break_usar_command_into_params.hh"
#include "get_topics_list.hh"

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

#define strNcmp(B,C) strncasecmp(B,C,strlen(C))

//#######################################################################
//  USARGazebo Plugin Option Reader
//#######################################################################

#define MAX_StartPoses 10
#define MAX_StartPose_Length 1024

#ifndef numof
#define numof(X) (sizeof(X)/sizeof(typeof(X[0])))
#endif

char Plugin_Option_Name_StartPose[][30]
              ={"StartPose_1", "StartPose_2", "StartPose_3"
               ,"StartPose_4", "StartPose_5", "StartPose_6"
               ,"StartPose_7", "StartPose_8", "StartPose_9"};

struct Plugin_Option_Parameters
{
  int Num_Of_StartPoses;
  char StartPoses[MAX_StartPoses][MAX_StartPose_Length];
  Plugin_Option_Parameters(void):Num_Of_StartPoses(0) { }
  void Read_Option_Parameters(sdf::ElementPtr _sdf)
  {
    int i;
    for(i = 0; i < numof(Plugin_Option_Name_StartPose); i++)
    {
      if(_sdf->HasElement(Plugin_Option_Name_StartPose[i]))
      {
        std::stringstream s;
        s << _sdf->GetElement(Plugin_Option_Name_StartPose[i])
                    ->Get<std::string>();
        strncat(StartPoses[Num_Of_StartPoses], s.str().c_str()
                 , MAX_StartPose_Length);
        Num_Of_StartPoses++;
      }
    }
//    for(i = 0; i < Num_Of_StartPoses; i++)
//      printf("%d %s\n", i, StartPoses[i]); 
  }
  int Search_StartPose(char* _name, float& x, float& y, float& z
                                    ,float& roll, float& pitch, float& yaw)
  {
    int   i, read_fields;
    char  name[100];
    for(i = 0; i < Num_Of_StartPoses; i++)
    {
      printf("%d %s\n", i, StartPoses[i]); 
      read_fields = sscanf(StartPoses[i], "%s %f,%f,%f %f,%f,%f"
                            , name, &x, &y, &z, &roll, &pitch, &yaw);
      if(7 != read_fields)
        return 0; // Broken parameters.
      if(0 == strNcmp(_name, name))
        break; // Found
    }
    return 1; // Found and There were all parameters
  }
};

Plugin_Option_Parameters POP;

//#######################################################################
//  Command Server
//#######################################################################

//////////////////////////////////////////////////////////////////
// Useful flags 
struct RD
{
  char  Request;
  char  Done;
  void  Init(void) { }
  RD(void) : Request(0), Done(0) { Init(); }
};

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
  gazebo::transport::NodePtr    _node;
  // Add your own variables here
//  RD   Msg, Spawn;
  int                           robot_was_spawned;
  char                          model_name[100], own_name[100], topic_root[100];
  char*                         ucbuf; // a pointer of USARSim command string
  gazebo::math::Vector3         spawn_location;
  gazebo::math::Quaternion      spawn_direction;
  TopicsList                    topics_list;

  //////////////////////////////////////////////////////////////////
  // Initialize something...
  void Init(void)
  {
    model_name[0] = own_name[0] = topic_root[0] = 0;
  }

  //////////////////////////////////////////////////////////////////
  // USARcommand.Constructor
  USARcommand(Server_Framework<USARcommand>&parent) : 
    _parent(parent), _socket(_ioservice), ucbuf(NULL), robot_was_spawned(0)
      , topics_list()
//    , Msg(), Spawn() 
  { Init(); }

  //////////////////////////////////////////////////////////////////
  // USARcommand.Accept_Process
  void Accept_Process(void)
  {
    _node = gazebo::transport::NodePtr(new gazebo::transport::Node());
    _node->Init();
//std::cout << "Accept_Process a [" << this << "]" << std::endl;
/* SAMPLE CODE : Sendback Accepted Acknowledgment for debug
    boost::asio::streambuf  ack_comment;
    std::iostream           st(&ack_comment);
    st << "+ -- Accepted ["  << this << "]" << std::endl;;
    boost::asio::write(_socket, ack_comment);
*/
    while(1)
      UC_check_command_from_USARclient();
  }

  //////////////////////////////////////////////////////////////////
  // USARcommand.Send_SENS   ## UNDER CONSTRUCTION ##
  void Send_SENS(void)
  {
    static int  refresh_cycle = 0;
    if(1 != robot_was_spawned)
      return;
      // 1. Check robot's sensors from topics list
    if(NULL == topics_list.Search(own_name) || 0 == topics_list.Size()
      || 50 < refresh_cycle++)
    {
      refresh_cycle = 0;
      topics_list.Refresh_Topics_List();
      topics_list.Filter(own_name);
      if(0 == topics_list.Size())
        return;
    }
      // 2. send SENS of each sensors
    for(typename std::set<char*>::iterator i = topics_list._topics_list.begin()
            ; i != topics_list._topics_list.end(); i++)
    {
      // UNDER CONSTRUCTION!! Sensor keywords and functions are all fake!!
      if(NULL != strcasestr(*i, "scan"))
      {
      // Process_laser_sensor(*i);
      }
      else if(NULL != strcasestr(*i, "gps"))
      {
      // Process_gps_sensor(*i);
      }
    }
  }

  //////////////////////////////////////////////////////////////////
  // USARcommand.functions which need to be defined outside of this class definition
  void UC_check_command_from_USARclient(void);
};

enum USAR_Command_Error_Code
{
  UCE_GOOD                        = 0,
  UCE_INCLUDING_BROKEN_BRACE      = 1,
  UCE_MISSING_NECESSARY_PARAMETER = 2,
  UCE_DO_NOT_CALL_AFTER_SPAWNED   = 3
};

//////////////////////////////////////////////////////////////////
// UC_INIT
struct UC_INIT
{
  //////////////////////////////////////////////////////////////////
  //  Variables
  USARcommand& _parent;
  //////////////////////////////////////////////////////////////////
  //  The constructor
  UC_INIT(USARcommand& parent) : _parent(parent)
  {
      // Already a robot has been spawned, then return
    if(1 == _parent.robot_was_spawned)
      return;
    if(UCE_GOOD == read_params_from_usar_command())
      spawn_a_robot();
  }
  //////////////////////////////////////////////////////////////////
  //  read_params_from_usar_command
  int read_params_from_usar_command(void)
  {
    float                          x = 0, y = 0, z = 0, r = 0, p = 0, yaw = 0;
    char*                          rtn;
    Break_USAR_Command_Into_Params BUCIP(_parent.ucbuf);
//BUCIP.Disp();
      // Already a robot has been spawned, then return
    if(1 == _parent.robot_was_spawned)
      return UCE_DO_NOT_CALL_AFTER_SPAWNED;
    if(BIE_GOOD != BUCIP.Error_code())
      return UCE_INCLUDING_BROKEN_BRACE;
    rtn = BUCIP.Search("Location");
    if(NULL != rtn)
      sscanf(rtn, "%f,%f,%f", &x, &y, &z);
    rtn = BUCIP.Search("Rotation");
    if(NULL != rtn)
      sscanf(rtn, "%f,%f,%f", &r, &p, &yaw);
    rtn = BUCIP.Search("ClassName");
    if(NULL != rtn)
    {
      rtn = BUCIP.Search("Name");
      if(NULL != rtn)
        record_robot_param(BUCIP.Search("Name"), BUCIP.Search("ClassName")
          ,x, y, z, r, p, yaw);
      else
        record_robot_param(BUCIP.Search("ClassName"), BUCIP.Search("ClassName")
          ,x, y, z, r, p, yaw);
      return UCE_GOOD;
    }
    return UCE_MISSING_NECESSARY_PARAMETER;
//  Sample command : INIT {ClassName pioneer3at_with_sensors}{Name Robo_A}{Location 1,-2,0}{Rotation 0,0,0}{Start Point1}
  }
  //////////////////////////////////////////////////////////////////
  //  spawn_a_robot
  void spawn_a_robot(void)
  {
    char	model_cmd[100];
      // Already a robot has been spawned, then return
    if(1 == _parent.robot_was_spawned)
      return;
      // Create a publisher on the ~/factory topic
    gazebo::transport::PublisherPtr factoryPub
      = _parent._node->Advertise<gazebo::msgs::Factory>("~/factory");
      // Create the message
    gazebo::msgs::Factory msg;
      // Prepare command
    sprintf(model_cmd, "model://%s", _parent.model_name);
      // Model file to load
    msg.set_sdf_filename(model_cmd);
      // Set this robot's own name
    // msg.set_edit_name(_parent.own_name); //Please fix this function! > Dr.Nate
      // Pose to initialize the model to
    gazebo::msgs::Set(msg.mutable_pose()
      , gazebo::math::Pose(_parent.spawn_location, _parent.spawn_direction));
      // Send the message
    factoryPub->Publish(msg);
//  usleep(500); // Wait for finishing spawn job
//  checking loop to get topics of the robot
//  if any topics of the robot could be got, set _parent.robot_was_spawned 1.
    _parent.robot_was_spawned = 1;
  }
  //////////////////////////////////////////////////////////////////
  //  record_robot_param
  void record_robot_param(char* _own_name, char* _model_name
                     , float x, float y, float z, float q1, float q2, float q3)
  {
      // Already a robot has been spawned, then return
    if(1 == _parent.robot_was_spawned)
      return;
    gazebo::math::Vector3 _location(x, y, z);
    gazebo::math::Quaternion _direction(q1, q2, q3);
    sprintf(_parent.topic_root, "~/%s", _own_name);
//    sprintf(_parent.topic_root, "/gazebo/default/%s", _own_name);
    strcpy(_parent.own_name, _model_name);// This is tempolary method by set_edit_name() could be fixed
//    strcpy(_parent.own_name, _own_name); // This is correct , but now set_edit_name() was out of order, then we can not use this.
    strcpy(_parent.model_name, _model_name);
    _parent.spawn_location = _location;
    _parent.spawn_direction = _direction;
  }
};

//////////////////////////////////////////////////////////////////
// UC_DRIVE.DRIVE_TYPE
enum DRIVE_TYPE
{
  RT_DiffarentialDrive = 1,
  RT_SkidsteerDrive    = 2
};

//////////////////////////////////////////////////////////////////
// Definition of UC_DRIVE class
struct UC_DRIVE
{
  //////////////////////////////////////////////////////////////////
  //  Variables
  USARcommand& _parent;
  float        _left_power, _right_power;
  //////////////////////////////////////////////////////////////////
  //  The constructor
  UC_DRIVE(USARcommand& parent)
    : _parent(parent), _left_power(0), _right_power(0)
  {
    if(1 != _parent.robot_was_spawned)
      return;
    if(UCE_GOOD == read_params_from_usar_command())
      drive_a_robot();
  }
  //////////////////////////////////////////////////////////////////
  //  read_params_from_usar_command
  int read_params_from_usar_command(void)
  {
    char*                        rtn;
    Break_USAR_Command_Into_Params BUCIP(_parent.ucbuf);
//BUCIP.Disp();
    if(BIE_GOOD != BUCIP.Error_code())
      return UCE_INCLUDING_BROKEN_BRACE;
    rtn = BUCIP.Search("LEFT");
    if(NULL != rtn)
      sscanf(rtn, "%f", &_left_power);
    rtn = BUCIP.Search("RIGHT");
    if(NULL != rtn)
      sscanf(rtn, "%f", &_right_power);
    return UCE_GOOD;
//  Sample command : DRIVE {RIGHT 2.0}{LEFT 2.0}
  }
  //////////////////////////////////////////////////////////////////
  //  drive_a_robot
  void drive_a_robot(void)
  {
    char  _TopicName[100];
    float _speed = 0, _turn = 0;
    int   _RobotType = RT_SkidsteerDrive;
      // Already a robot has been spawned, then return
    if(1 != _parent.robot_was_spawned)
      return;
      // Prepare topic name for drive command
 // sprintf(_TopicName, "~/%s/vel_cmd", _parent.own_name);
    sprintf(_TopicName, "~/%s/vel_cmd", _parent.model_name);
      // Create a publisher on the ~/factory topic
    gazebo::transport::PublisherPtr _pub_vel_cmd 
      = _parent._node->Advertise<gazebo::msgs::Pose>(_TopicName);
      // Wait for finishing the connection
 // _pub_vel_cmd->WaitForConnection();
      // Calc speed and turn
    _speed = (_right_power + _left_power) / 2.0;
    _turn  = -_right_power + _left_power;
      // Adjust parameters by robot drive type
    switch(_RobotType)
    {
      case RT_DiffarentialDrive:
        // Pioneer 2DX(Skidsteer); Turn > 0 then robot turn clock wise
        //_turn = _turn;
        break;
      case RT_SkidsteerDrive   : 
        // Pioneer 3AT(Diffarential); Turn < 0 then robot turn clock wise
        _turn = -_turn;
        break;
    }
      // Set the message
    gazebo::math::Pose pose(_speed, 0, 0, 0, 0, _turn);
    gazebo::msgs::Pose msg;
    gazebo::msgs::Set(&msg, pose);
      // Send the message
    _pub_vel_cmd->Publish(msg);
  }
};

//////////////////////////////////////////////////////////////////
//  UC_GETSTARTPOSES
//   Defined on USARSim Manual P.36-45
struct UC_GETSTARTPOSES
{
  //////////////////////////////////////////////////////////////////
  //  Variables
  USARcommand& _parent;
  int          NumOfStartPoses;
  //////////////////////////////////////////////////////////////////
  //  The constructor
  UC_GETSTARTPOSES(USARcommand& parent) : _parent(parent)
  {
    int                    i;
    int                    nwritten;
    boost::asio::streambuf response;
    std::iostream          st_response(&response);
//printf("%d", POP.Num_Of_StartPoses);
    st_response << "NFO {StartPoses " << POP.Num_Of_StartPoses << "}";
    for(i = 0; i < POP.Num_Of_StartPoses; i++)
      st_response << "{" << POP.StartPoses[i] << "}";
    st_response << std::endl;
//std::cout << st_response;
    nwritten = boost::asio::write(_parent._socket, response);
    if(nwritten <= 0) {
      perror("NFO message could not be written, check errno");
    }
/*  else if (nwritten != response.length()) {
      perror("not whole NFO message written");
    }*/
    // Should also convert the Quaternion spawn_direction
    // Could StartPoses be defined in a Gazebo map?
  }
};

//////////////////////////////////////////////////////////////////
//  USAR Command fetch    ## DO NOT MOVE THIS FUNCTION FROM HERE ##
void USARcommand::UC_check_command_from_USARclient(void)
{
  int nread;
  boost::system::error_code err;
    // Get an USAR command string
  nread = boost::asio::read_until(_socket, _buffer , "\r\n", err);
//std::cout << "Child_Session_Loop a [" << this << "]" << std::endl;
    // error routine : anyone write this with boost....
  if(nread < 0)
  {
    perror("Could not read socket 3000");
//  close(_socket); // I do not know the "close" function in boost library.
    _parent.Remove_Child_Session(this); // from child session list
                                        //  but can I do this here really?
    _thread.join();
  } 
  else if(nread == 0) 
  {
    perror("EOF, should break connection");
//  close(_socket); // I do not know the "close" function in boost library.
    _parent.Remove_Child_Session(this); // from child session list
                                        //  but can I do this here really?
    _thread.join();
  }
    // Make a copy of the USAR command string for keeping it local
  std::iostream st(&_buffer);
  std::stringstream s;
  s << st.rdbuf();
  if(NULL != ucbuf)
    delete ucbuf;
  ucbuf = new char[strlen((char*)s.str().c_str())+2];
  strcpy(ucbuf, (char*)s.str().c_str());
// DEBUG information output  
//printf("COMMAND string = %s\n", ucbuf );
    // Call each USAR command programs
  if(0 == strNcmp(ucbuf, "INIT"))
    UC_INIT UC_init(*this);
  else if(0 == strNcmp(ucbuf, "DRIVE"))
    UC_DRIVE UC_drive(*this);
  else if(0 == strNcmp(ucbuf, "GETSTARTPOSES"))
    UC_GETSTARTPOSES UC_getstartposes(*this);
  delete ucbuf;
  ucbuf = NULL;
}

//#######################################################################
//  Image Server
//#######################################################################

//////////////////////////////////////////////////////////////////
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

//////////////////////////////////////////////////////////////////
// USARimage : A structure for transferring image data
struct USARimage
{
  //////////////////////////////////////////////////////////////////
  // USARimage.Variables
  Server_Framework<USARimage>     &_parent;
  boost::asio::io_service          _ioservice;
  boost::asio::ip::tcp::socket     _socket;
  boost::asio::streambuf           _buffer;
  boost::thread                    _thread;
  gazebo::transport::NodePtr       _node;
  gazebo::transport::SubscriberPtr _sub_camera_image;
  // Add your own variables here
  int  flag_OK, flag_U;
  char model_name[100], own_name[100], topic_camera[100];
  void send_full_size_image(ConstImageStampedPtr& _msg);
  void send_rectangle_area_image(ConstImageStampedPtr& _msg);
  void imageserver_callback(ConstImageStampedPtr& _msg);
  // A part of sample code to decide camera topic name automatically
  TopicsList topics_list;

  //////////////////////////////////////////////////////////////////
  // Initialize some thing..
  void Init(void) { }

  //////////////////////////////////////////////////////////////////
  // USARimage.Constructor
  USARimage(Server_Framework<USARimage>&parent)
    : _parent(parent), _socket(_ioservice), flag_OK(0), flag_U(0)
      , topics_list()
  { Init(); }

  //////////////////////////////////////////////////////////////////
  // USARimage.Child_Session_Loop_Core
  void Child_Session_Loop_Core(void)
  {
    boost::system::error_code err;
    boost::asio::read_until(_socket, _buffer , "\r\n", err);
      // OK or U command should be checked here`
    std::iostream st(&_buffer);
    std::stringstream s;
    s << st.rdbuf();
    if(0 == strNcmp(s.str().c_str(), "OK"))
    {
      flag_OK = 1;
    }
    else if(0 == strNcmp(s.str().c_str(), "U["))
    {
      flag_U = 1;
    }
/* SAMPLE CODE : Display received data for debug
    std::iostream st(&_buffer);
    std::stringstream s;
    s << st.rdbuf();
    std::cout << CString(s.str().c_str()) << std::endl;
    st << s.str() << std::endl; */
// SAMPLE CODE : Sendback received data for debug 
//    boost::asio::write(_socket, _buffer);
  }

  //////////////////////////////////////////////////////////////////
  // USARimage.Accept_Process
  void Accept_Process(void)
  {
    _node = gazebo::transport::NodePtr(new gazebo::transport::Node());
    _node->Init();
      // Set topic name of camera
    // You can change camera by each child session, 
    // if we could change the hand-shaking rule of sending cameara image data.
    // For example...
    // After connecting, send camera topic name like following at first at once.
    // "~/pioneer3at_with_sensors/chassis/r_camera/image"
    /* We can get the sent camera topic name by following codes,but it's a future protocol.
    boost::system::error_code err;
    boost::asio::read_until(_socket, _buffer, "\r\n", err);
    std::iostream st(&_buffer);
    std::stringstream s;
    s << st.rdbuf();
    sprintf(topic_camera, "%s", s.str().c_str());
    */
      // Now we can not get camera topic name by current hand-shaking rule
      // , then I wrote following line instead.
/* Sample Code for using topics list to decide cameara topic name automatically
    if(NULL == topics_list.Search((char*)"image"))
    {
      topics_list.Get_Topics_List();
      topics_list.Filter(own_name);
      if(NULL == topics_list.Search((char*)"image"))
        return;
    }
    sprintf(topic_camera, "%s"
                      , topics_list.Search((char*)"image"));
*/
    sprintf(topic_camera, "%s"
                      , "~/pioneer3at_with_sensors/chassis/r_camera/image");
    _sub_camera_image 
      = _node->Subscribe(topic_camera, &USARimage::imageserver_callback, this);
    while(1)
      Child_Session_Loop_Core();
  }
};

//////////////////////////////////////////////////////////////////
// Function is called everytime a message is received on topics
void USARimage::imageserver_callback(ConstImageStampedPtr& _msg)
{
  if(flag_OK)
    send_full_size_image(_msg);
  else if(flag_U)
    send_rectangle_area_image( _msg);
}

//////////////////////////////////////////////////////////////////
// Send camera image
void USARimage::send_full_size_image(ConstImageStampedPtr& _msg)
{
  flag_OK = 0;
  int            ImageSize = 3 * _msg->image().width() * _msg->image().height();
  unsigned char* ip = (unsigned char*)_msg->image().data().c_str();
  unsigned char* ImageBuf = new unsigned char[5 + ImageSize];
    // Set Image format type <= 0
  ImageBuf[0] = 0; // Raw Data
    // Set imagesize
  *(unsigned long int*)&ImageBuf[1] = ImageSize; // Image Size
    // Repacking into USARImage Raw Format
  for(int i = 0; i < _msg->image().width() * _msg->image().height(); i++)
  {
    ImageBuf[5+i*3+0] /*Blue */ = ip[i*3+2]/*Blue*/;
    ImageBuf[5+i*3+1] /*Red  */ = ip[i*3+0]/*Red*/;
    ImageBuf[5+i*3+2] /*Green*/ = ip[i*3+1]/*Green*/;
  }
    // Send USAR Image Data
  boost::asio::write(_socket,boost::asio::buffer((void*)ImageBuf,5+ImageSize));
    // Free ImageBuf
  delete ImageBuf;
/* For checking to be able to get an image data
  static int filenumber=0;
  char filename[100];
  if(filenumber>10)
    return;
  sprintf(filename, "./tmp%02d.PPM", (filenumber++)%10);
  SaveAsPPM(filename, _msg);
*/
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

  public: USARGazebo(void):UCp(NULL),UIp(NULL),counter1(0) { }
  virtual ~USARGazebo()
  {
    connections.clear();
    if(NULL != UCp)
      delete UCp;
    if(NULL != UIp)
      delete UIp;
  }

  ///////////////////////////////////////////////
  // \brief Load the robocup rescue plugin
  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
  {
    world = _parent;
    Init();
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
    // Read option parameters of this plugin from world file.
    POP.Read_Option_Parameters(_sdf);
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
    if(NULL == UCp)
      UCp = new Server_Framework<USARcommand>(3000);
    if(NULL == UIp)
      UIp = new Server_Framework<USARimage>(5003);
  }

private:
  /// \brief Gazebo communication node
  gazebo::transport::NodePtr node;
  /// \brief Gazebo factory publisher
  gazebo::transport::PublisherPtr factoryPub;
  /// \brief Gazebo subscriber to the ~/usarsim topic.
  /// This is a stand-in for the usarsim interface
  gazebo::transport::SubscriberPtr usarsimSub;
  /// \brief Keep a pointer to the world.
  gazebo::physics::WorldPtr world;
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
