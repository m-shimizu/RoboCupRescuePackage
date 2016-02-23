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
// Return device type in topic name
const char* get_string_between_slash_from_topic_name(char* dist, 
       int dist_size, int slash_num_from_right, const char* topic_name)
{
  const char* i;
  const char* start;
  const char* end;
  int         copy_size;
  if(1 > slash_num_from_right)
    return (const char*)NULL;
  i = topic_name;
  i+= strlen(topic_name);
  end = i;
  start = (const char*)NULL;
  for(i--; topic_name <= i; i--)
  {
    if('/' == *i)
    {
      start = (i + 1);
      slash_num_from_right--;
      if(0 < slash_num_from_right)
      {
        end = i;
        start = (const char*)NULL;
        continue;
      }
      break;
    }
  }
  if((const char*)NULL == start)
    return (const char*)NULL;
  dist_size--; // for 0x0(end of string)
  copy_size = (int)(end - start);
  strncpy(dist, start, (copy_size > dist_size)?dist_size:copy_size);
  dist[copy_size] = 0;
  return (const char*)dist;
}

//////////////////////////////////////////////////////////////////
// Return device type in topic name
const char* get_type_from_topic_name(char* dist, int dist_size, 
                                              const char* topic_name)
{
  return get_string_between_slash_from_topic_name(dist, dist_size, 1
                                                          , topic_name);
}
//////////////////////////////////////////////////////////////////
// Return device name in topic name
const char* get_name_from_topic_name(char* dist, int dist_size, 
                                              const char* topic_name)
{
  return get_string_between_slash_from_topic_name(dist, dist_size, 2
                                                          , topic_name);
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
  int Search_StartPose(char* target_name, float& _x, float& _y, float& _z,
                       float& _roll, float& _pitch, float& _yaw)
  {
    int   i, read_fields;
    char  name[100];
    float x, y, z, roll, pitch, yaw;
    for(i = 0; i < Num_Of_StartPoses; i++)
    {
      printf("%d %s\n", i, StartPoses[i]); 
      read_fields = sscanf(StartPoses[i], "%s %f,%f,%f %f,%f,%f"
                            , name, &x, &y, &z, &roll, &pitch, &yaw);
      if(7 != read_fields)
        return 0; // Broken parameters.
      if(0 == strNcmp(target_name, name))
      {
        _x     = x;
        _y     = y;
        _z     = z;
        _roll  = roll;
        _pitch = pitch;
        _yaw   = yaw;
        return 1; // Found and There were all parameters
      }
    }
    return 0; // No same name parameter.
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
// ROBOT_TYPE
enum ROBOT_TYPE
{
  RT_GROUNDVECHILE = 1,
  RT_LEGGEDROBOT   = 2,
  RT_NAUTICVEHICLE = 3,
  RT_AERIALVEHICLE = 4
};

//////////////////////////////////////////////////////////////////
// ROBOT_TYPE_NAME
#define RT_D_GROUNDVEHICLE "GroundVehicle"
#define RT_D_LEGGEDROBOT   "LeggedRobot"
#define RT_D_NAUTICVEHICLE "NauticVehicle"
#define RT_D_AERIALVEHICLE "AerialVehicle"
#define RT_D_UNKNOWN       "Unknown"

//////////////////////////////////////////////////////////////////
// DRIVE_TYPE for UC_DRIVE
enum DRIVE_TYPE
{
  DT_DiffarentialDrive = 1,
  DT_SkidsteerDrive    = 2,
  DT_QuadRotor         = 3
};

//////////////////////////////////////////////////////////////////
// Robot Database 
struct ROBOT_DATABASE
{
  const char* Name;
  const char* Type;
  const char* SteeringType;
  const float Mass;
  const float MaxSpeed;
  const float MaxTorque;
  const float MaxFrontSteer;
  const float MaxRearSteer;
  int         Battery;
  int         DriveType;
} Robot_DB[]
 = {{"pioneer3at_with_sensors", RT_D_GROUNDVEHICLE, "SkidSteered",
   //Mass Speed Torque Front Rear Battery  DT
       20,    5,    10,   10,  10,   3600, DT_SkidsteerDrive}

   ,{"quadrotor", RT_D_AERIALVEHICLE, "RotaryWing",
   //Mass Speed Torque Front Rear Battery  DT
       20,    5,    10,   10,  10,   3600, DT_QuadRotor}

   ,{"noname", RT_D_UNKNOWN, "Unknown",
   //Mass Speed Torque Front Rear Battery  DT
        0,    0,     0,    0,   0,      0, 0} };

int get_number_of_Robot_DB(const char* robot_name)
{
  int i;
  for(i = 0; (numof(Robot_DB) - 1) > i; i++)
    if(0 == strNcmp(robot_name, Robot_DB[i].Name))
      break;
  return i;
}

#define get_type_of_robot(X) Robot_DB[get_number_of_Robot_DB(X)].Type
#define get_steeringtype_of_robot(X) Robot_DB[get_number_of_Robot_DB(X)]\
           .SteeringType
#define get_mass_of_robot(X) Robot_DB[get_number_of_Robot_DB(X)]\
           .Mass
#define get_maxspeed_of_robot(X) Robot_DB[get_number_of_Robot_DB(X)]\
           .MaxSpeed
#define get_maxtorque_of_robot(X) Robot_DB[get_number_of_Robot_DB(X)]\
           .MaxTorque
#define get_maxfrontsteer_of_robot(X) Robot_DB[get_number_of_Robot_DB(X)]\
           .MaxFrontSteer
#define get_maxrearsteer_of_robot(X) Robot_DB[get_number_of_Robot_DB(X)]\
           .MaxRearSteer
#define get_battery_of_robot(X) Robot_DB[get_number_of_Robot_DB(X)].Battery
#define get_drive_of_robot(X) Robot_DB[get_number_of_Robot_DB(X)].DriveType

//////////////////////////////////////////////////////////////////
// EFFECTER_TYPE_NAME
#define ET_D_G_RFID         "Unknown"
#define ET_D_U_RFID         "RFID"
#define ET_D_G_GPS          "gps"
#define ET_D_U_GPS          "GPS"
#define ET_D_G_IMU          "imu"
#define ET_D_U_IMU          "INS"
#define ET_D_G_CAMERA       "image"
#define ET_D_U_CAMERA       "Camera"
#define ET_D_G_LASERSCANNER "scan"
#define ET_D_U_LASERSCANNER "RangeScanner"
#define ET_D_G_ODOMETRY     "Odometry_from_IMU"
#define ET_D_U_ODOMETRY     "Odometry"

//////////////////////////////////////////////////////////////////
// Gazebo-USARSim Database 
struct Gazebo_USARSim_DATABASE
{
  const char* Gazebo_name;
  const char* USARSim_name;
} Gazebo_USARSim_DB[]
 = {{ET_D_G_LASERSCANNER, ET_D_U_LASERSCANNER},
    {ET_D_G_CAMERA,       ET_D_U_CAMERA},
    {ET_D_G_RFID,         ET_D_U_RFID},
    {ET_D_G_GPS,          ET_D_U_GPS},
    {ET_D_G_IMU,          ET_D_U_IMU},
    {"Unknown",           "UnKnown"} };

int get_number_of_DB_From_Gazebo_name(const char* _name)
{
  int i;
  for(i = 0; (numof(Gazebo_USARSim_DB) - 1) > i; i++)
    if(0 == strNcmp(_name, Gazebo_USARSim_DB[i].Gazebo_name))
      break;
  return i;
}

int get_number_of_DB_From_USARSim_name(const char* _name)
{
  int i;
  for(i = 0; (numof(Gazebo_USARSim_DB) - 1) > i; i++)
    if(0 == strNcmp(_name, Gazebo_USARSim_DB[i].USARSim_name))
      break;
  return i;
}

#define get_USARSim_name_of(X) \
      Gazebo_USARSim_DB[get_number_of_DB_From_Gazebo_name(X)].USARSim_name
#define get_Gazebo_name_of(X) \
      Gazebo_USARSim_DB[get_number_of_DB_From_USARSim_name(X)].Gazebo_name

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
  int                           STA_Last_sec, STA_Count_Per_1sec, 
                                STA_Cut_Counter;

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
      , STA_Last_sec(0), STA_Count_Per_1sec(0), STA_Cut_Counter(0)
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
    boost::asio::streambuf response;
    std::ostream           os_res(&response);
    sprintf(Current_time_in_form, "%.2f", Current_time);
    if(STA_Last_sec != Current_sec)
    {
      STA_Count_Per_1sec = 0;
      remain_battery--;
    }
    if(STA_Cut_Counter < 0)
    {
      STA_Cut_Counter = 100; // Display STA 1 times per 100 loop times
      os_res << "STA " << 
        "{Type " << get_type_of_robot(model_name) << "}" << 
        "{Time " << Current_time_in_form << "}" << 
        "{Battery " << remain_battery << "}" << 
        "{CountPer1sec " << STA_Count_Per_1sec << "}" << 
        "\r\n";
    }
    else
      STA_Cut_Counter--;
    STA_Count_Per_1sec++;
    STA_Last_sec = Current_sec;
    boost::asio::write(_socket, response);
  }

  //////////////////////////////////////////////////////////////////
  // Prototypes of callback functions
  void Process_laser_scanner_callback(ConstLaserScanStampedPtr& _msg);
  void Process_gps_callback(ConstGPSPtr& _msg);
  void Process_imu_callback(ConstIMUPtr& _msg);
  void Send_odometry(ConstIMUPtr& _msg);
  
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
    //registered_topics_list.Disp_topics_list();
    // 3. Register callback functions for sensors
      for(typename std::set<char*>::iterator i 
                                 = current_topics_list._topics_list.begin()
            ; i != current_topics_list._topics_list.end(); i++)
      {
        // At only first time, register a callback function for the sensor.
        // RangeFinder 
        if(NULL != strcasestr(*i, ET_D_G_LASERSCANNER))
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
        else if(NULL != strcasestr(*i, ET_D_G_GPS))
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
        else if(NULL != strcasestr(*i, ET_D_G_IMU))
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

#define GET_TYPE_FROM_TOPIC(T,X) get_type_from_topic_name(T, sizeof(T),\
                                         registered_topics_list.Search(X))
#define GET_NAME_FROM_TOPIC(T,X) get_name_from_topic_name(T, sizeof(T),\
                                         registered_topics_list.Search(X))

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
  int   angles;
  char  tmpbuf[100];
  float angle_width = _msg->scan().angle_max() - _msg->scan().angle_min();
  float steps       = angle_width / _msg->scan().angle_step();
  float resolution  = angle_width / steps;
  os << "SEN {Type RangeScanner}" <<
            "{Name " << GET_NAME_FROM_TOPIC(tmpbuf, "scan") << "}" <<
            "{Resolution " << resolution << "}" << 
            "{FOV " << angle_width << "}" << 
            "{Range ";
  /*
  for(typename std::set<double>::iterator i=_msg->scan().ranges().begin()
        ; i != _msg->scan().ranges().end(); i++)
  */
  angles = _msg->scan().ranges_size();
  if(0 < angles)
  {
    int    i;
    float* angle_values = new float[angles];
    for(i = 0; angles > i; i++)
      angle_values[i] = _msg->scan().ranges(i);
    for(i = 0; angles > i; i++)
      os << angle_values[(angles - 1)-i] << (((angles - 1) > i)?",":"");
    delete angle_values;
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
  char  tmpbuf[100];
  boost::asio::streambuf sen;
  std::ostream os(&sen);
  iLat = (int)_msg->latitude_deg();
  iLon = (int)_msg->longitude_deg();
  fLat = (_msg->latitude_deg() - iLat) * 60.0;
  fLon = (_msg->longitude_deg() - iLon) * 60.0;
  os << "SEN {Type GPS}" <<
            "{Name " << GET_TYPE_FROM_TOPIC(tmpbuf, "gps") << "}" <<
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
  char   tmpbuf[100];
  static gazebo::math::Vector3 pose(0,0,0), vel(0,0,0), acl;
  double w, x, y, z, sqw, sqx, sqy, sqz, yaw, pitch, roll;
  float  Current_time = _parent._world->GetRealTime().Double();
  float  dt;
  static float Last_time = 0;
  const gazebo::msgs::Quaternion& orientation = _msg->orientation();
  const gazebo::msgs::Vector3d& linear_acceleration = 
                                        _msg->linear_acceleration();
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
  yaw   = atan2(2.0 * (x*y + z*w), (sqx - sqy - sqz + sqw));
  pitch = atan2(2.0 * (y*z + x*w), (-sqx - sqy + sqz + sqw));
  roll  = asin(-2.0 * (x*z - y*w));
  // Calc Pose
  dt  = Current_time - Last_time;
  Last_time = Current_time;
  if(dt < 1)
  {
    acl.x = linear_acceleration.x();
    acl.y = linear_acceleration.y();
    acl.z = linear_acceleration.z();
    // Reverse Rotation around Z axis
    float cz = cos(yaw), sz = sin(yaw);
    gazebo::math::Vector3 racl;
    racl.x = cz * acl.x - sz * acl.y;
    racl.y = sz * acl.x + cz * acl.y;
    // Integration
    vel.x += racl.x * dt;
    vel.y += racl.y * dt;
    vel.z +=  acl.z * dt;
    pose.x += vel.x * dt;
    pose.y += vel.y * dt;
    pose.z += vel.z * dt;
//printf("IMU: X,Y,YAW=%f,%f,%f\n", pose.x, pose.y, yaw);
  }
  boost::asio::streambuf sen;
  std::ostream os(&sen);
  os << "SEN {Type INS}" <<
        "{Name " << GET_NAME_FROM_TOPIC(tmpbuf, "imu") << "}" <<
        "{Location " << pose.x << "," << pose.y << "," << pose.z << "}" << 
        "{Orientation " << roll << "," << pitch << "," << yaw << "}";
  os << "\r\n"; 
  boost::asio::write(_socket, sen);
//  std::cout << _msg->DebugString();
  ///////////////////////////////////////////////////////////////
  // Send Odometory 
  boost::asio::streambuf sen_odo;
  std::ostream os_odo(&sen_odo);
  os_odo << "SEN {Type Odometry}" <<
        "{Name " << ET_D_G_ODOMETRY << "}" <<
        "{Pose " << pose.x << "," << pose.y << "," << yaw << "}";
  os_odo << "\r\n"; 
  boost::asio::write(_socket, sen_odo);
}

//////////////////////////////////////////////////////////////////
// Error Code Definition for UC_**** functions
enum USAR_Command_Error_Code
{
  UCE_GOOD                        = 0,
  UCE_INCLUDING_BROKEN_BRACE      = 1,
  UCE_MISSING_NECESSARY_PARAMETER = 2,
  UCE_DO_NOT_CALL_AFTER_SPAWNED   = 3,
  UCE_NO_EFFECTER                 = 4
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
    // If a Start parameter was given....(Option)
    rtn = BUCIP.Search("Start");
    if(NULL != rtn)
      POP.Search_StartPose(rtn, x, y, z, r, p, yaw);
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
//Please fix msg.set_edit_name()  > Dr.Nate
//  msg.set_edit_name(_parent.own_name);
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
//                          (GOOD TEMPLATE TO ADD ANOTHER COMMAND)
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
    char*                          rtn;
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
      case DT_DiffarentialDrive:
        // Pioneer 2DX(Skidsteer); Turn > 0 then robot turn clock wise
        //_turn = _turn;
        break;
      case DT_SkidsteerDrive   : 
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
    st_response << "\r\n";
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

#define UC_GET_TYPE_FROM_TOPIC(T,X) get_type_from_topic_name(T, sizeof(T),\
                                 _parent.registered_topics_list.Search(X))
#define UC_GET_NAME_FROM_TOPIC(T,X) get_name_from_topic_name(T, sizeof(T),\
                                 _parent.registered_topics_list.Search(X))

#define UC_GET_TYPE_FROM_TL(T,X,I) \
           get_type_from_topic_name(T,sizeof(T),tl.Search_n(I,X))
#define UC_GET_NAME_FROM_TL(T,X,I) \
           get_name_from_topic_name(T,sizeof(T),tl.Search_n(I,X))

//////////////////////////////////////////////////////////////////
// UC_GETGEO
//  GETGEO is defined in the USARSim manual P.77
//  GEO is define in the USARSim manual P.58 - P.61
struct UC_GETGEO
{
  //////////////////////////////////////////////////////////////////
  //  Variables
  USARcommand& _parent;

  //////////////////////////////////////////////////////////////////
  //  The constructor
  UC_GETGEO(USARcommand& parent) : _parent(parent)
  {
    if(1 != _parent.robot_was_spawned)
      return;
    read_params_from_usar_command();
  }

  //////////////////////////////////////////////////////////////////
  //  GEO_set_effecters_params
  int GEO_set_effecters_params(std::iostream& st_response, 
                                             const char* _USARSim_name)
  {
    TopicsList  tl;
    int         effecters;
    char        tmpbuf[100];
    int         flag_gps = 0;
    const char* Gazebo_name = get_Gazebo_name_of(_USARSim_name);
    Break_USAR_Command_Into_Params BUCIP(_parent.ucbuf);
    const char* effecter_name = BUCIP.Search("Name");
    st_response << "{Type " << _USARSim_name << "}";
    if(0 == strNcmp(Gazebo_name, ET_D_G_GPS))
      flag_gps = 1;
    tl.Get_Topics_List();
    if(0 == tl.Filter(_parent.own_name))
      return UCE_NO_EFFECTER;
    if(0 == (effecters = tl.Filter(Gazebo_name)))
      return UCE_NO_EFFECTER;
    if(NULL != effecter_name && 0 == (effecters=tl.Filter(effecter_name)))
      return UCE_NO_EFFECTER;
    for(int i = 0; effecters > i; i++)
    {
      st_response << 
        "{Name " << 
          ((flag_gps)?(UC_GET_TYPE_FROM_TL(tmpbuf, Gazebo_name, i)): 
                      (UC_GET_NAME_FROM_TL(tmpbuf, Gazebo_name, i))) << 
        " Location " << "0,0.1,0.1" <<
        " Orientation " << "0,0.1,0.1" << 
        " Mount " << _parent.own_name << "}";
    }
    return UCE_GOOD;
  }

  //////////////////////////////////////////////////////////////////
  //  GEO_set_odometry_params
  int GEO_set_odometry_params(std::iostream& st_response, 
                                             const char* _USARSim_name)
  {
    st_response << "{Type " << _USARSim_name << "}";
    st_response << 
      "{Name " << ET_D_G_ODOMETRY << 
      " Location " << "0,0.1,0.1" <<
      " Orientation " << "0,0.1,0.1" << 
      " Mount " << _parent.own_name << "}";
    return UCE_GOOD;
  }

  //////////////////////////////////////////////////////////////////
  //  read_params_from_usar_command
  int read_params_from_usar_command(void)
  {
    char*                          rtn;
    char                           tmpbuf[100];
    Break_USAR_Command_Into_Params BUCIP(_parent.ucbuf);
    boost::asio::streambuf         response;
    std::iostream                  st_response(&response);
    int                            nwritten;
//  BUCIP.Disp();
    if(BIE_GOOD != BUCIP.Error_code())
      return UCE_INCLUDING_BROKEN_BRACE;
    rtn = BUCIP.Search("Type");
    if(NULL != rtn)
    {
      st_response << "GEO ";
      if(NULL != strcasestr(rtn, ET_D_U_RFID))
        GEO_set_effecters_params(st_response, ET_D_U_RFID);
      else if(NULL != strcasestr(rtn, ET_D_U_CAMERA))
        GEO_set_effecters_params(st_response, ET_D_U_CAMERA);
      else if(NULL != strcasestr(rtn, ET_D_U_LASERSCANNER))
        GEO_set_effecters_params(st_response, ET_D_U_LASERSCANNER);
      else if(NULL != strcasestr(rtn, ET_D_U_GPS))
        GEO_set_effecters_params(st_response, ET_D_U_GPS);
      else if(NULL != strcasestr(rtn, ET_D_U_IMU))
        GEO_set_effecters_params(st_response, ET_D_U_IMU);
      else if(NULL != strcasestr(rtn, ET_D_U_ODOMETRY))
        GEO_set_odometry_params(st_response, ET_D_U_ODOMETRY);
      else if(NULL != strcasestr(rtn, "Robot"))
      {
        st_response << 
          "{Type " << get_type_of_robot(_parent.model_name) << "}" << 
          "{Name " << _parent.own_name << "}" <<
          "{Dimensions 0.7744,0.6318,0.5754}" << 
          "{COG 0.0000,0.0000,-0.1000}" <<
          "{WheelRadius 0.1922}" << 
          "{WheelSeparation 0.5120}" << 
          "{Wheelbase 0.3880}";
      }
      st_response << "\r\n";
      nwritten = boost::asio::write(_parent._socket, response);
      if(0 >= nwritten)
        perror("GEO message could not be written, check errno");
    }
    return UCE_GOOD;
  }
};

//////////////////////////////////////////////////////////////////
// UC_GETCONF
//  GETCONF is defined in the USARSim manual P.77
//  CONF is define in the USARSim manual P.61 - P.64
struct UC_GETCONF
{
  //////////////////////////////////////////////////////////////////
  //  Variables
  USARcommand& _parent;

  //////////////////////////////////////////////////////////////////
  //  The constructor
  UC_GETCONF(USARcommand& parent) : _parent(parent)
  {
    if(1 != _parent.robot_was_spawned)
      return;
    read_params_from_usar_command();
  }

  //////////////////////////////////////////////////////////////////
  //  CONF_set_effecters_params
  int CONF_set_effecters_params(std::iostream& st_response, 
                                             const char* _USARSim_name)
  {
    TopicsList  tl;
    int         effecters;
    char        tmpbuf[100];
    int         flag_gps = 0;
    const char* Gazebo_name = get_Gazebo_name_of(_USARSim_name);
    Break_USAR_Command_Into_Params BUCIP(_parent.ucbuf);
    const char* effecter_name = BUCIP.Search("Name");
    st_response << "{Type " << _USARSim_name << "}";
    if(0 == strNcmp(Gazebo_name, ET_D_G_GPS))
      flag_gps = 1;
    tl.Get_Topics_List();
    if(0 == tl.Filter(_parent.own_name))
      return UCE_NO_EFFECTER;
    if(0 == (effecters = tl.Filter(Gazebo_name)))
      return UCE_NO_EFFECTER;
    if(NULL != effecter_name && 0 == (effecters=tl.Filter(effecter_name)))
      return UCE_NO_EFFECTER;
    for(int i = 0; effecters > i; i++)
    {
      st_response << 
        "{Name " << 
          ((flag_gps)?(UC_GET_TYPE_FROM_TL(tmpbuf, Gazebo_name, i)): 
                      (UC_GET_NAME_FROM_TL(tmpbuf, Gazebo_name, i))) << 
//        " Location " << "0,0.1,0.1" <<
//        " Orientation " << "0,0.1,0.1" << 
//        " Mount " << _parent.own_name << 
        "}";
    }
    return UCE_GOOD;
  }

  //////////////////////////////////////////////////////////////////
  //  CONF_set_odometry_params
  int CONF_set_odometry_params(std::iostream& st_response, 
                                             const char* _USARSim_name)
  {
    st_response << "{Type " << _USARSim_name << "}";
    st_response << 
      "{Name " << ET_D_G_ODOMETRY << 
//      " Location " << "0,0.1,0.1" <<
//      " Orientation " << "0,0.1,0.1" << 
//      " Mount " << _parent.own_name << 
      "}";
    return UCE_GOOD;
  }

  //////////////////////////////////////////////////////////////////
  //  read_params_from_usar_command
  int read_params_from_usar_command(void)
  {
    char*                          rtn;
    char                           tmpbuf[100];
    Break_USAR_Command_Into_Params BUCIP(_parent.ucbuf);
    boost::asio::streambuf         response;
    std::iostream                  st_response(&response);
    int                            nwritten;
//  BUCIP.Disp();
    if(BIE_GOOD != BUCIP.Error_code())
      return UCE_INCLUDING_BROKEN_BRACE;
    rtn = BUCIP.Search("Type");
    if(NULL != rtn)
    {
      st_response << "CONF ";
      if(NULL != strcasestr(rtn, ET_D_U_RFID))
        CONF_set_effecters_params(st_response, ET_D_U_RFID);
      else if(NULL != strcasestr(rtn, ET_D_U_CAMERA))
        CONF_set_effecters_params(st_response, ET_D_U_CAMERA);
      else if(NULL != strcasestr(rtn, ET_D_U_LASERSCANNER))
        CONF_set_effecters_params(st_response, ET_D_U_LASERSCANNER);
      else if(NULL != strcasestr(rtn, ET_D_U_GPS))
        CONF_set_effecters_params(st_response, ET_D_U_GPS);
      else if(NULL != strcasestr(rtn, ET_D_U_IMU))
        CONF_set_effecters_params(st_response, ET_D_U_IMU);
      else if(NULL != strcasestr(rtn, ET_D_U_ODOMETRY))
        CONF_set_odometry_params(st_response, ET_D_U_ODOMETRY);
      else if(NULL != strcasestr(rtn, "Robot"))
      {
        if(NULL != strcasestr(get_type_of_robot(_parent.model_name),
                                                 RT_D_GROUNDVEHICLE))
        {
          st_response << 
            "{Type " << get_type_of_robot(_parent.model_name) << "}" << 
            "{Name " << _parent.own_name << "}" <<
            "{SteeringType " << 
               get_steeringtype_of_robot(_parent.model_name) << "}" << 
            "{Mass " << 
               get_mass_of_robot(_parent.model_name) << "}" << 
            "{MaxSpeed " << 
               get_maxspeed_of_robot(_parent.model_name) << "}" << 
            "{MaxTorque " << 
               get_maxtorque_of_robot(_parent.model_name) << "}" << 
            "{MaxFrontSteer " << 
               get_maxfrontsteer_of_robot(_parent.model_name) << "}" << 
            "{MaxRearSteer " << 
               get_maxrearsteer_of_robot(_parent.model_name) << "}";
        }
      }
      st_response << "\r\n";
      nwritten = boost::asio::write(_parent._socket, response);
      if(0 >= nwritten)
        perror("GEO message could not be written, check errno");
    }
    return UCE_GOOD;
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
//  line.getline(is);
    std::getline(is, line);
    if(0 < line.length())
    {
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
      else if(0 == strNcmp(ucbuf, "GETGEO"))
        UC_GETGEO UC_getgeo(*this);
      else if(0 == strNcmp(ucbuf, "GETCONF"))
        UC_GETCONF UC_getconf(*this);
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
printf("image\n");
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
printf("Sent a full size image in raw data, size = %d\n", 5+ImageSize);
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
