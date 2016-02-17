// Desctiption: 
// This program's a translator between USARSim protocol and Gazebo protocol.
// This file is a primaly file of this program. 
// You can add some codes for treating USARsim command which you need in this file by copying and editing alread existing other codes of USARSim protcol.
// UC_INIT class is a good sample code for adding a new code for treating a new USARSim command.
//
// Author : Prof.Arnoud Visser, Dr.Nate Koenig, Masaru Shimizu
// E-Mail : shimizu@sist.chukyo-u.ac.jp
// Date   : Mar.2015-Feb.2016
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

#ifndef numof
#define numof(X) (sizeof(X)/sizeof(typeof(X[0])))
#endif

//#######################################################################
//  USARGazebo Plugin Option Reader
//#######################################################################

#define MAX_StartPoses 10
#define MAX_StartPose_Length 1024

char Plugin_Option_Name_StartPose[][30]
              ={"StartPose_1", "StartPose_2", "StartPose_3"
               ,"StartPose_4", "StartPose_5", "StartPose_6"
               ,"StartPose_7", "StartPose_8", "StartPose_9"};

//////////////////////////////////////////////////////////////////
// Return own name in topic name
const char* right_of_last_slash_in_topic_name(const char* topic_name)
{
	const char* i;
	i = topic_name;
	i+= strlen(topic_name);
	for(i--; topic_name <= i; i--)
	{
		if('/' == *i)
		{
			i++;
			break;
		}
	}
	if(topic_name + strlen(topic_name) == i)
		return (const char*)NULL;
	return i;
}

//////////////////////////////////////////////////////////////////
// Plugin Optoin Parameters Reader from SDF 
struct Plugin_Option_Parameters
{
  int Num_Of_StartPoses;
  char StartPoses[MAX_StartPoses][MAX_StartPose_Length];

  //////////////////////////////////////////////////////////////////
  // Constructor
  Plugin_Option_Parameters(void) : Num_Of_StartPoses(0) { }

  //////////////////////////////////////////////////////////////////
  // Read Option Parameters from SDF file
	void Read_Option_Parameters(sdf::ElementPtr _sdf)
	{
    Read_Start_Pose_Parameters(_sdf);
	}

  //////////////////////////////////////////////////////////////////
  // Read Start Pose Parameters into StartPose array
  void Read_Start_Pose_Parameters(sdf::ElementPtr _sdf)
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
  }

  //////////////////////////////////////////////////////////////////
  // Search Start Pose from StartPose array and set parameters 
  int Search_StartPose(char* target_name, float& x, float& y, float& z
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
      if(0 == strNcmp(target_name, name))
        break; // Found
    }
    return 1; // Found and There were all parameters
  }
};

Plugin_Option_Parameters POP;

//#######################################################################
//  USAR Command Server
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
// DRIVE_TYPE for UC_DRIVE
enum DRIVE_TYPE
{
  RT_DiffarentialDrive = 1,
  RT_SkidsteerDrive    = 2,
  RT_QuadRotor         = 3
};

//////////////////////////////////////////////////////////////////
// Robot Database 
struct ROBOT_DATABASE
{
  const char* Name;
  const char* Type;
  int         Battery;
  int         DriveType;
} Robot_DB[]
 = {{"pioneer3at_with_sensors", "GroundVehicle", 3600, RT_SkidsteerDrive}
   ,{"quadrotor", "AerialVehicle", 3600, RT_QuadRotor}
   ,{"noname", "Unknown", 10, 0} };

int get_number_of_Robot_DB(char* robot_name)
{
  int i;
  for(i = 0; (numof(Robot_DB) - 1) > i; i++)
    if(0 == strNcmp(robot_name, Robot_DB[i].Name))
      break;
  return i;
}

#define get_type_of_robot(X) Robot_DB[get_number_of_Robot_DB(X)].Type
#define get_battery_of_robot(X) Robot_DB[get_number_of_Robot_DB(X)].Battery
#define get_drive_of_robot(X) Robot_DB[get_number_of_Robot_DB(X)].DriveType

namespace gazebo
{

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
//RD   Msg, Spawn;
  int                           robot_was_spawned;
  char                          model_name[100], 
                                own_name[100], 
                                topic_root[100];
  char*                         ucbuf; // Pointer of USARSim command string
  gazebo::math::Vector3         spawn_location;
  gazebo::math::Quaternion      spawn_direction;
  TopicsList                    current_topics_list,registered_topics_list;
  int                           full_battery, remain_battery;
  gazebo::transport::SubscriberPtr* _sub;
  std::vector<event::ConnectionPtr> connections;

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
      , current_topics_list(), registered_topics_list()
//    , Msg(), Spawn() 
  { Init(); }

  //////////////////////////////////////////////////////////////////
  // USARcommand.Desstructor
  ~USARcommand() 
	{
    connections.clear();
  }

  //////////////////////////////////////////////////////////////////
  // USARcommand.Process_STA for USARcommand.Send_SENS
  void Process_STA(void)
  {
    float      Current_time = _parent._world->GetRealTime().Double();
    int        Current_sec = (int)Current_time;
    char       Current_time_in_form[50];
    static int Last_sec, Count_Per_1sec;
    boost::asio::streambuf response;
    std::ostream           os_res(&response);
    sprintf(Current_time_in_form, "%.2f", Current_time);
    if(Last_sec != Current_sec)
    {
      Count_Per_1sec = 0;
      remain_battery--;
    }
    os_res << "STA {Type " << get_type_of_robot(model_name) << "}"
           << "{Time " << Current_time_in_form << "}"
           << "{Battery " << remain_battery << "}"
           << "{CountPer1sec " << Count_Per_1sec << "}"
           << "\r\n";
    Count_Per_1sec++;
    Last_sec = Current_sec;
    boost::asio::write(_socket, response);
  }

  //////////////////////////////////////////////////////////////////
  // Prototypes of callback functions
  void Process_laser_scanner_callback(ConstLaserScanStampedPtr& _msg);
  void Process_gps_callback(ConstGPSPtr& _msg);
  void Process_imu_callback(ConstIMUPtr& _msg);
	
  //////////////////////////////////////////////////////////////////
  // USARcommand.Send_SENS   ## NEED YOUR HELP TO INCREASE SENSORS ##
  void Send_STA_and_Search_Sensors_and_Register_Callbacks(void)
  {
    static int             refresh_cycle = 0;
    if(1 != robot_was_spawned)
      return;
    // 1. send STA of this robot
    Process_STA();
    // 2. Check robot's sensors from topics list
    if(NULL == current_topics_list.Search(own_name) 
      ||  0 == current_topics_list.Size()
      || 700 < refresh_cycle++)
    {
      refresh_cycle = 0;
      current_topics_list.Refresh_Topics_List();
      current_topics_list.Filter(own_name);
      if(0 == current_topics_list.Size())
        return;
//    registered_topics_list.Disp_topics_list();
    // 3. Register callback functions for sensors
      for(typename std::set<char*>::iterator i 
                                 = current_topics_list._topics_list.begin()
            ; i != current_topics_list._topics_list.end(); i++)
      {
        // At only first time, register a callback function for the sensor.
        // RangeFinder 
        if(NULL != strcasestr(*i, "scan"))
        {
          if(NULL == registered_topics_list.Search(*i))
          {
std::cout << "Laser cb registered" << std::endl;
            registered_topics_list.Add_a_topics(*i);
	          _sub = new gazebo::transport::SubscriberPtr;
		// The function Subscribe needs a valiable which be assigned 
		//  a return value from the function Subscribe. Without the assigning,
		//   a call-back function  which was registered 
		//    by the function Subscribe will NOT be call-backed.
		// DO NOT REMOVE *_sub
            *_sub = _node->Subscribe((const char*)*i
               , &USARcommand::Process_laser_scanner_callback, this);
          }
        }
        // GPS 
        else if(NULL != strcasestr(*i, "gps"))
        {
          if(NULL == registered_topics_list.Search(*i))
          {
std::cout << "GPS cb registered" << std::endl;
            registered_topics_list.Add_a_topics(*i);
	          _sub = new gazebo::transport::SubscriberPtr;
		// The function Subscribe needs a valiable which be assigned 
		//  a return value from the function Subscribe. Without the assigning,
		//   a call-back function  which was registered 
		//    by the function Subscribe will NOT be call-backed.
		// DO NOT REMOVE *_sub
            *_sub = _node->Subscribe((const char*)*i
               , &USARcommand::Process_gps_callback, this);
          }
        }
        // IMU 
        else if(NULL != strcasestr(*i, "imu"))
        {
          if(NULL == registered_topics_list.Search(*i))
          {
std::cout << "IMU cb registered" << std::endl;
            registered_topics_list.Add_a_topics(*i);
	          _sub = new gazebo::transport::SubscriberPtr;
		// The function Subscribe needs a valiable which be assigned 
		//  a return value from the function Subscribe. Without the assigning,
		//   a call-back function  which was registered 
		//    by the function Subscribe will NOT be call-backed.
		// DO NOT REMOVE *_sub
            *_sub = _node->Subscribe((const char*)*i
               , &USARcommand::Process_imu_callback, this);
          }
        }
      }
    }
  }

  //////////////////////////////////////////////////////////////////
  // USARcommand.functions which need to be defined 
  //  at outside of this class definition
  void UC_check_command_from_USARclient(void);

  //////////////////////////////////////////////////////////////////
  // USARcommand.Accept_Process
  void Accept_Process(void)
  {
    _node = gazebo::transport::NodePtr(new gazebo::transport::Node());
    _node->Init(/*_parent._world->GetName()*/);
		// Binding a sensor functions with cyclic timer
    connections.push_back(event::Events::ConnectWorldUpdateBegin(
      boost::bind(&USARcommand
			    ::Send_STA_and_Search_Sensors_and_Register_Callbacks, this)));
//std::cout << "Accept_Process address [" << this << "]" << std::endl;
    while(1)
      UC_check_command_from_USARclient();
  }
};

//////////////////////////////////////////////////////////////////////
// Process_laser_scanner_callback
//   Defined on USARSim Manual P.53-54
// This function is called everytime a message is received on topics
//  See following page or source files about type of _msg
//   /usr/include/gazebo-5.1/gazebo/msgs/laserscan.pb.h
//   https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1sensors_1_1GpuRaySensor.html
//   https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1sensors_1_1RaySensor.html
//   gazebo-5.0.1/gazebo/sensors/Raysensor.{cc|hh}
//   gazebo-5.0.1/gazebo/msgs/sensor.proto
// 
//  BUT FINALLY, I FOUND THE CORRECT DATA TYPE FROM:
//   gz topic -i /gazebo/default/pioneer3at_with_sensors/laser_tilt_mount_link/laser_tilt/scan
//
// About ConstLaserScanStampedPtr
//   https://www.google.co.jp/webhp?sourceid=chrome-instant&ion=1&espv=2&ie=UTF-8#q=ConstLaserScanStampedPtr
//   http://docs.ros.org/indigo/api/gazebo_plugins/html/classgazebo_1_1GazeboRosLaser-members.html
//   http://docs.ros.org/indigo/api/gazebo_plugins/html/classgazebo_1_1GazeboRosLaser.html#af46b29ad8afebfb8d96edcda9ecc58ad
//   http://docs.ros.org/indigo/api/gazebo_plugins/html/gazebo__ros__gpu__laser_8cpp_source.html
//#include <gazebo/physics/World.hh>
//#include <gazebo/physics/HingeJoint.hh>
//#include <gazebo/sensors/Sensor.hh>
//#include <gazebo/common/Exception.hh>
//#include <gazebo/sensors/GpuRaySensor.hh>
//#include <gazebo/sensors/SensorTypes.hh>
//#include <gazebo/transport/transport.hh>
//#include <tf/tf.h>
//#include <tf/transform_listener.h>
void USARcommand::Process_laser_scanner_callback(
                                          ConstLaserScanStampedPtr& _msg)
{
  boost::asio::streambuf sen;
  std::ostream os(&sen);
	float angle_width = _msg->scan().angle_max() - _msg->scan().angle_min();
  float steps       = angle_width / _msg->scan().angle_step();
	float resolution  = steps / angle_width;
	os << "SEN {Type RangeScanner}" <<
            "{Name ????}" << 
//	          "{Name " << right_of_last_slash_in_topic_name(topic_name)
//						???}" << 
	          "{Resolution " << resolution << "}" << 
						"{FOV " << angle_width << "}" << 
						"{Range ";
	/*
  for(typename std::set<double>::iterator i=_msg->scan().ranges().begin()
        ; i != _msg->scan().ranges().end(); i++)
  */
	for(int i = 0; _msg->scan().ranges_size() > i; i++)
	{
    os << _msg->scan().ranges(i);
		if(_msg->scan().ranges_size() > i + 1)
			os << ",";
	}
	os << "}";
	os << "\r\n"; 
  boost::asio::write(_socket, sen);
//  std::cout << _msg->DebugString();
	/*
  _msg->time().sec();
  _msg->time().nsec();
  _msg->scan().angle_min();
  _msg->scan().angle_max();
  _msg->scan().angle_step();
  _msg->scan().range_min();
  _msg->scan().range_max();
  _msg->scan().ranges_size();
  _msg->scan().ranges(i)
  _msg->scan().intensities_size();
  _msg->scan().intensities(i)
	*/
}

//////////////////////////////////////////////////////////////////////
// Process_gps_callback
//   Defined on USARSim Manual P.54-55
// This function is called everytime a message is received on topics
//  See following page or source files about type of _msg
// About ConstGPSPtr
//   https://github.com/arpg/Gazebo/blob/master/gazebo/msgs/gps.proto
//    message GPS
//    {
//      required Time time                    = 1;
//      required string link_name             = 2;
//      required double latitude_deg          = 3;
//      required double longitude_deg         = 4;
//      required double altitude              = 5;
//      optional double velocity_east         = 6;
//      optional double velocity_north        = 7;
//      optional double velocity_up           = 8;
//    }
void USARcommand::Process_gps_callback(ConstGPSPtr& _msg)
{
	int   iLon, iLat;
	float fLon, fLat;
  boost::asio::streambuf sen;
  std::ostream os(&sen);
	iLat = (int)_msg->latitude_deg();
	iLon = (int)_msg->longitude_deg();
	fLat = (_msg->latitude_deg() - iLat) * 60.0;
	fLon = (_msg->longitude_deg() - iLon) * 60.0;
	os << "SEN {Type GPS}" <<
            "{Name ????}" << 
            "{Latitude " << iLat << "," << fLat << ",N}" << 
            "{Lonitude " << iLon << "," << fLon << ",E}" << 
						"{Satellites 8}{Fix 1}";
	os << "\r\n"; 
  boost::asio::write(_socket, sen);
//  std::cout << _msg->DebugString();
	/*
  _msg->time().sec();
  _msg->time().nsec();
  _msg->latitude_deg();
  _msg->longitude_deg();
  _msg->altitude();
  _msg->velocity_east();
  _msg->velocity_north();
  _msg->velocity_up();
	*/
}

//////////////////////////////////////////////////////////////////////
// Process_imu_callback
//   Defined on USARSim Manual P.55-56
// This function is called everytime a message is received on topics
//  See following page or source files about type of _msg
//   https://github.com/UWARG/simSPIKE/blob/master/GazeboToArduino/imutest
//   https://github.com/arpg/Gazebo/blob/master/gazebo/msgs/imu.proto
//    message IMU
//    {
//      required Time stamp                   = 1;
//      required string entity_name           = 2;
//      required Quaternion orientation       = 3;
//      required Vector3d angular_velocity    = 4;
//      required Vector3d linear_acceleration = 5;
//    }
void USARcommand::Process_imu_callback(ConstIMUPtr& _msg)
{
  double w, x, y, z, sqw, sqx, sqy, sqz, yaw, pitch, roll;
  const gazebo::msgs::Quaternion &orientation = _msg->orientation();
  _msg->stamp().sec();
  _msg->stamp().nsec();
  //Break out the values from the Quarternion and convert to YPR
  w = orientation.w();
  x = orientation.x();
  y = orientation.y();
  z = orientation.z();
  sqw = w*w;    
  sqx = x*x;    
  sqy = y*y;    
  sqz = z*z; 
   //YPR conversions from Quarternion
  yaw   = atan2(2.0 * (x*y + z*w), (sqx - sqy - sqz + sqw)) 
           * (180.0f/M_PI);
  pitch = atan2(2.0 * (y*z + x*w), (-sqx - sqy + sqz + sqw)) 
           * (180.0f/M_PI);          
  roll  = asin(-2.0 * (x*z - y*w)) * (180.0f/M_PI);
  boost::asio::streambuf sen;
  std::ostream os(&sen);
	os << "SEN {Type INS}" <<
            "{Name ????}" << 
            "{Location "<<x<<","<<y<<","<<z<<"}"<< 
            "{Orientation "<<roll<<","<<pitch<<","<<yaw<<"}";
	os << "\r\n"; 
  boost::asio::write(_socket, sen);
//  std::cout << _msg->DebugString();
}

//////////////////////////////////////////////////////////////////
// Error Code Definition for UC_**** functions
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
    float                          x = 0, y = 0, z = 0, 
                                   r = 0, p = 0, yaw = 0;
    int                            battery = 0;
    char*                          rtn;
    Break_USAR_Command_Into_Params BUCIP(_parent.ucbuf);
    // Display Init parameters for debug.
    BUCIP.Disp();
    // Already a robot has been spawned, then return
    if(1 == _parent.robot_was_spawned)
      return UCE_DO_NOT_CALL_AFTER_SPAWNED;
    // If there are any error in received init paramers, return error code.
    if(BIE_GOOD != BUCIP.Error_code())
      return UCE_INCLUDING_BROKEN_BRACE;
    // Read Robot ClassName parameter (Required)
    rtn = BUCIP.Search("ClassName");
    if(NULL == rtn)
      return UCE_MISSING_NECESSARY_PARAMETER; // FATAL ERROR: NO CLASSNAME
    // If a location parameter was given....(Option)
    rtn = BUCIP.Search("Location");
    if(NULL != rtn)
      sscanf(rtn, "%f,%f,%f", &x, &y, &z);
    // If a rotation parameter was given....(Option)
    rtn = BUCIP.Search("Rotation");
    if(NULL != rtn)
      sscanf(rtn, "%f,%f,%f", &r, &p, &yaw);
    // If a battery parameter was given....(Option)
    rtn = BUCIP.Search("Battery");
    if(NULL != rtn)
      sscanf(rtn, "%d", &battery);
    else // If no battery parameter, get it from Robot_DB
      battery = get_battery_of_robot(BUCIP.Search("ClassName"));
    // Read own robot name parameter (Option) AND set parameters
    rtn = BUCIP.Search("Name");
    if(NULL != rtn)
      record_robot_param(BUCIP.Search("Name")
        , BUCIP.Search("ClassName"), x, y, z, r, p, yaw, battery);
    else
      record_robot_param(BUCIP.Search("ClassName")
        , BUCIP.Search("ClassName"), x, y, z, r, p, yaw, battery);
    return UCE_GOOD;
  }

  //////////////////////////////////////////////////////////////////
  //  spawn_a_robot
  void spawn_a_robot(void)
  {
    char  model_cmd[100];
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
//  msg.set_edit_name(_parent.own_name); Please fix this function! > Dr.Nate
    // Pose to initialize the model to
    gazebo::msgs::Set(msg.mutable_pose()
     , gazebo::math::Pose(_parent.spawn_location, _parent.spawn_direction));
    // Send the message
    factoryPub->Publish(msg);
    usleep(1000); // Wait for finishing spawn job
    //  checking loop to get topics of the robot
    //   if any topics of the robot could be got, 
    //    set _parent.robot_was_spawned with "1".
    _parent.robot_was_spawned = 1;
  }

  //////////////////////////////////////////////////////////////////
  //  record_robot_param
  void record_robot_param(char* _own_name, char* _model_name
    , float x, float y, float z, float q1, float q2, float q3, int battery)
  {
      // Already a robot has been spawned, then return
    if(1 == _parent.robot_was_spawned)
      return;
    gazebo::math::Vector3 _location(x, y, z);
    gazebo::math::Quaternion _direction(q1, q2, q3);
    sprintf(_parent.topic_root, "~/%s", _own_name);
//  sprintf(_parent.topic_root, "/gazebo/default/%s", _own_name);
    // The following line is tempolary method
    //  until set_edit_name() would be fixed
    strcpy(_parent.own_name, _model_name /*_own_name*/);
    // The following line is correct
    // , but now set_edit_name() was out of order, then we can not use this.
    //strcpy(_parent.own_name, _own_name); 
    strcpy(_parent.model_name, _model_name);
    _parent.spawn_location = _location;
    _parent.spawn_direction = _direction;
    _parent.full_battery = _parent.remain_battery = battery;
  }
};

//////////////////////////////////////////////////////////////////
// UC_DRIVE
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
//  BUCIP.Disp();
    if(BIE_GOOD != BUCIP.Error_code())
      return UCE_INCLUDING_BROKEN_BRACE;
    rtn = BUCIP.Search("LEFT");
    if(NULL != rtn)
      sscanf(rtn, "%f", &_left_power);
    rtn = BUCIP.Search("RIGHT");
    if(NULL != rtn)
      sscanf(rtn, "%f", &_right_power);
    return UCE_GOOD;
  }

  //////////////////////////////////////////////////////////////////
  //  drive_a_robot
  void drive_a_robot(void)
  {
    char  _TopicName[100];
    float _speed = 0, _turn = 0;
    int   _DriveType = get_drive_of_robot(_parent.model_name);
    // Already a robot has been spawned, then return
    if(1 != _parent.robot_was_spawned)
      return;
    // Prepare topic name for drive command
    sprintf(_TopicName, "~/%s/vel_cmd", _parent.own_name);
 // sprintf(_TopicName, "~/%s/vel_cmd", _parent.model_name);
    // Create a publisher on the ~/factory topic
    gazebo::transport::PublisherPtr _pub_vel_cmd 
      = _parent._node->Advertise<gazebo::msgs::Pose>(_TopicName);
    // Wait for finishing the connection
 // _pub_vel_cmd->WaitForConnection();
    // Calc speed and turn
    _speed = (_right_power + _left_power) / 2.0;
    _turn  = -_right_power + _left_power;
    // Adjust parameters by robot drive type
    switch(_DriveType)
    {
      case RT_DiffarentialDrive:
        // Pioneer 2DX(Skidsteer); Turn > 0 then robot turn clock wise
        //_turn = _turn;
        break;
      case RT_SkidsteerDrive   : 
        // Pioneer 3AT(Diffarential); Turn < 0 then robot turn clock wise
        _turn = -_turn;
        break;
      case 0:
        return;
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
//  printf("%d", POP.Num_Of_StartPoses);
    st_response << "NFO {StartPoses " << POP.Num_Of_StartPoses << "}";
    for(i = 0; i < POP.Num_Of_StartPoses; i++)
      st_response << "{" << POP.StartPoses[i] << "}";
    st_response << std::endl;
//  std::cout << st_response;
    nwritten = boost::asio::write(_parent._socket, response);
    if(0 >= nwritten)
    {
      perror("NFO message could not be written, check errno");
    }
/*  else if (nwritten != response.length()) {
      perror("not whole NFO message written");
    }*/
    // Should also convert the Quaternion spawn_direction.
    // Could StartPoses be defined in a Gazebo map?
		//  No, but they can be written as options of USARGazebo plugin in sdf.
  }
};

//////////////////////////////////////////////////////////////////
//  USAR Command fetch    ## DO NOT MOVE THIS FUNCTION FROM HERE ##
void USARcommand::UC_check_command_from_USARclient(void)
{
  int nread;
  // error routine : anyone write this with boost....
  boost::system::error_code err;
  // Get an USAR command string
  nread = boost::asio::read_until(_socket, _buffer, "\r\n", err);
  // DO NOT USE _buffer, including just reading _buffer.
	//  Ex) After reading _buffer, values were erased in _buffer.
//std::cout << "Child_Session_Loop address [" << this << "]" << std::endl;
//std::cout << "nread = " << nread << std::endl;
  if(0 > nread)
  {
    perror("Could not read socket 3000");
    _parent.Remove_Child_Session(this);
    _thread.join();
  } 
  else if(0 == nread) 
  {
    perror("EOF, should break connection");
    _parent.Remove_Child_Session(this);
    _thread.join();
  }
  std::istream is(&_buffer);
  std::string line;
  for(;1;)
  {
    // Copy 1 line by 1 line from _buffer into "line"
		//  to keep the USAR command string values
    std::getline(is, line);
    if(0 < line.length())
    {
      // TEST CODE MEMO : socket output
      /*
      boost::asio::write(_socket, boost::asio::buffer(line.c_str()
        , line.length()));
      boost::asio::write(_socket, boost::asio::buffer(line));
			*/
      if(NULL != ucbuf)
        delete ucbuf;
      ucbuf = new char[line.length()+2];
      strcpy(ucbuf, (char*)line.c_str());
      // Display DEBUG information
      printf("Received command string = %s\n", ucbuf);
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
    else
      break;
  }
}

//#######################################################################
//  Image Server
//#######################################################################

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
  int  flag_OK, flag_U, flag_SS;
  char model_name[200], own_name[200], topic_camera[300];
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
    : _parent(parent), _socket(_ioservice)
     , flag_OK(0), flag_U(0), flag_SS(0)
      , topics_list()
  { Init(); }

  //////////////////////////////////////////////////////////////////
  // USARimage.Child_Session_Loop_Core
  void Child_Session_Loop_Core(void)
  {
    boost::system::error_code err;
    boost::asio::read_until(_socket, _buffer , "\r\n", err);
  // DO NOT USE _buffer, including just reading _buffer.
	//  Ex) After reading _buffer, values were erased in _buffer.
//  std::cout << "Child_Session_Loop_Core address ["<<this<<"]"<<std::endl;
    // OK or U command should be checked here.
		//  U is underconstruction, not working.
		// SS is for debug.
    std::istream is(&_buffer);
    std::string  line;
    for(;1;)
    {
      std::getline(is, line);
      if(0 < line.length())
      {
        if(0 == strNcmp(line.c_str(), "OK"))
          flag_OK = 1;
        else if(0 == strNcmp(line.c_str(), "U["))
          flag_U = 1;
        else if(0 == strNcmp(line.c_str(), "SS"))
          flag_SS = 1;
      }
      else
        break;
    }
  }

  //////////////////////////////////////////////////////////////////
  // USARimage.Accept_Process
  void Accept_Process(void)
  {
    _node = gazebo::transport::NodePtr(new gazebo::transport::Node());
    _node->Init();
    // Set topic name of camera
		// Now the camera topic name is set as : 
    // "~/pioneer3at_with_sensors/chassis/r_camera/image" .
    // You can change camera by each child session, 
    //  if we could change the hand-shaking rule of sending 
    //   cameara image data.
    // After connecting, the camera topic name can be tranfered 
		//  by codes like following, but it's a future protocol.
		/*
    boost::system::error_code err;
    boost::asio::read_until(_socket, _buffer, "\r\n", err);
    std::istream is(&_buffer);
    std::string  line;
    for(;1;)
    {
      std::getline(is, line);
      if(0 < line.length())
      {
        if(0 == strNcmp(line.c_str(), "TP"))
        sprintf(camera_topic_name, "TP %s", line.c_str());
      }
      else
        break;
    }
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
    sprintf(camera_topic_name, "%s"
                      , topics_list.Search((char*)"image"));
*/
    sprintf(topic_camera, "%s"
                     , "~/pioneer3at_with_sensors/chassis/r_camera/image");
		// The function Subscribe needs a valiable which be assigned 
		//  a return value from the function Subscribe. Without the assigning,
		//   a call-back function  which was registered 
		//    by the function Subscribe will NOT be call-backed.
		// DO NOT REMOVE _sub_camera_image
    _sub_camera_image 
      = _node->Subscribe(topic_camera, &USARimage::imageserver_callback
                                                                  , this);
    while(1)
      Child_Session_Loop_Core();
  }
};

//////////////////////////////////////////////////////////////////
// SaveAsPPM saves an image for debug
//  See following page about ConstImageStampedPtr
//  https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1gui_1_1ImageView.html#a7c1d7a627bb39bfd216b1bef4e45035d
void  SaveAsPPM(char* filename, ConstImageStampedPtr &_msg)
{
  unsigned char* ip = (unsigned char*)_msg->image().data().c_str();
  FILE* fp = fopen(filename, "wt");
  fprintf(fp, "P3\n%d %d\n255\n", _msg->image().width()
                                               , _msg->image().height());
  for(int i=0;i<_msg->image().width()*_msg->image().height();i++)
  fprintf(fp, "%3d %3d %3d\n", ip[i*3], ip[i*3+1], ip[i*3+2]);
  fclose(fp);
}

//////////////////////////////////////////////////////////////////
// Function is called everytime a message is received on topics
void USARimage::imageserver_callback(ConstImageStampedPtr& _msg)
{
  if(flag_OK)
    send_full_size_image(_msg);
  else if(flag_U)
    send_rectangle_area_image( _msg);
  else if(flag_SS)
  {
    flag_SS = 0;
    SaveAsPPM((char*)"/tmp/GazeboCameraImage.ppm", _msg);
  }
}

//////////////////////////////////////////////////////////////////
// Send camera image
void USARimage::send_full_size_image(ConstImageStampedPtr& _msg)
{
  flag_OK = 0;
  int ImageSize = 3 * _msg->image().width() * _msg->image().height();
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
  boost::asio::write(_socket, boost::asio::buffer((void*)ImageBuf
                     , 5 + ImageSize));
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
	// Please anyone, write this function instead of me.....
}

//#######################################################################
//  World Plugin
//#######################################################################

  class USARGazebo : public WorldPlugin
  {

  /////////////////////////////////////////////
  /// Constructor
  public: USARGazebo(void):UCp(NULL),UIp(NULL) { }

  /////////////////////////////////////////////
  /// Destructor
  virtual ~USARGazebo()
  {
    if(NULL != UCp)
      delete UCp;
    if(NULL != UIp)
      delete UIp;
  }

  ///////////////////////////////////////////////
  // Load the robocup rescue plugin
  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
  {
    world = _parent;
    Init();
    POP.Read_Option_Parameters(_sdf);
  }

  /////////////////////////////////////////////
  // Called once after Load
  private: void Init()
  {
    if(NULL == UCp)
      UCp = new Server_Framework<USARcommand>(3000, world);
    if(NULL == UIp)
      UIp = new Server_Framework<USARimage>(5003, world);
  }

private:
  /// Keep a pointer to the world.
  gazebo::physics::WorldPtr world;
public:
  Server_Framework<USARcommand> *UCp;
  Server_Framework<USARimage>   *UIp;
  };
  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(USARGazebo)
}
