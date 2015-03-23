#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"

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
        char    Request;
        char    Done;
        void    Init(void) {Request = Done = 0;}
        RD(void) {Init();}
};

struct USARcommand
{
    int socket;
    RD  Msg, Spawn;
    int Spawned;
    char model_name[100], own_name[100], topic_root[100];
    gazebo::math::Vector3 spawn_location;
    gazebo::math::Quaternion spawn_direction;
    void Init(void) {Spawned = 0; Msg.Init(); Spawn.Init();}
    USARcommand(void) {Init();}
};

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

#define MAX_COMMAND_CLIENTS      16 // Depend on USARsim Manual P.50
USARcommand       rbuf[MAX_COMMAND_CLIENTS];
int             rbuf_num=0;

//#######################################################################
//  INIT Command

void UC_INIT_spawn_a_robot(USARcommand* rbuf_ptr)
{
    char	model_cmd[100];
    // Already a robot has been spawned, then return
    if(0 != rbuf_ptr->Spawned)
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
    sprintf(model_cmd, "model://%s", rbuf_ptr->model_name);
    // Model file to load
    msg.set_sdf_filename(model_cmd);
 // msg.set_edit_name(rbuf_ptr->own_name);
    // Pose to initialize the model to
    gazebo::msgs::Set(msg.mutable_pose()
        , gazebo::math::Pose(rbuf_ptr->spawn_location
            , rbuf_ptr->spawn_direction));
    // Send the message
    factoryPub->Publish(msg);
    // Set Spawned flag
    rbuf_ptr->Spawned = 1;
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

void UC_INIT_set_spawn_param(USARcommand* rbuf_ptr, char* own_name, char* model_name,
   float x, float y, float z, float q1, float q2, float q3)
{
    gazebo::math::Vector3 _location(x, y, z);
    gazebo::math::Quaternion _direction(q1, q2, q3);
    sprintf(rbuf_ptr->topic_root, "~/%s", rbuf_ptr->own_name);
    strcpy(rbuf_ptr->model_name, model_name);
    strcpy(rbuf_ptr->own_name, own_name);
    rbuf_ptr->spawn_location = _location;
    rbuf_ptr->spawn_direction = _direction;
}

//#######################################################################
//  GETSTARTPOSES Command

void UC_GETSTARTPOSES_give_start_poses(USARcommand* rbuf_ptr)
{
 // Defined on USARSim Manual P.43
 char response[128]; // should be enough
 int  nwritten, bytes;
 //  perror("we should send the string NFO {StartPoses #} {Name1 x,y,z a,b,c} over the socket");
    sprintf(response, "NFO {StartPoses 1}{PlayerStart %g, %g, %g 0,0,0}", rbuf_ptr->spawn_location[0], rbuf_ptr->spawn_location[1], rbuf_ptr->spawn_location[2]);
    // should also convert the Quaternion spawn_direction
    // Could there be multiple StartPoses?
    // Could StartPoses be defined in a Gazebo map?
    bytes = strlen(response);
    nwritten = write(rbuf_ptr->socket, response, bytes);

    if(nwritten <= 0) {
        perror("NFO message could not be written, check errno");
    } else if (nwritten != bytes) {
        perror("not whole NFO message written");
    }
}

//#######################################################################
//  USAR Command event loop

int check_command_from_USARclient(USARcommand* rbuf_ptr)
{
//  1.read 1 line from rbuf_ptr->socket
//  2.recognize a command
//  3.read parameters and set it into rbuf_ptr
//  4.if need, set or reset flag
    int nread;
    char line[4096]; // USARCommander uses a buffer length of 4K

    nread = read(rbuf_ptr->socket, line, 4096);

    if(nread < 0) {
	perror("Could not read socket 3000");
        close(rbuf_ptr->socket); 
	pthread_detach(pthread_self());
    } else if(nread == 0) {
	perror("EOF, should break connection");
        close(rbuf_ptr->socket);
// could not perform rbuf_num--, because there can be robots with a higher number
	pthread_detach(pthread_self());
    }

    if(strncmp(line,"INIT",4) == 0) {
       
        if(0 == rbuf_ptr->Spawned)
        {
            UC_INIT_set_spawn_param(rbuf_ptr, (char*)"Robo_A"
                , (char*)"pioneer3at_with_sensors", 1, -2, 0, 0, 0, 0);
            return UC_INIT;
        }
    } else if(strncmp(line,"GETSTARTPOSES",13) == 0) {
//      perror("GETSTARTPOSES, should give rbuf_ptr->spawn_location");
        return UC_GETSTARTPOSES;

    } else {
        return UC_NOOP;
    }
}

void *usarcommand_client_connection(void* rbuf_ptr)
{
    USARcommand*  rp = (USARcommand*)rbuf_ptr;
SCK_TXT(rp->socket, (char*)"USARGazebo:Message from child_connection\n");
    while(1)
    {
        switch(check_command_from_USARclient(rp))
        {
        case UC_INIT : UC_INIT_spawn_a_robot(rp);
                       break;
        case UC_GETSTARTPOSES : UC_GETSTARTPOSES_give_start_poses(rp);
                       break;
        }
    }
    return 0;
}

void *usarcommand_accept_loop(void* dummy)
{
    pthread_t          thread_hnd;
    struct sockaddr_in server, client;
    int                c, portal_socket;
    portal_socket = socket(AF_INET, SOCK_STREAM, 0);
    if(-1 == portal_socket)
    {
        perror("Could not create socket\n");
        exit(-1);
    }
    server.sin_family      = AF_INET;
    server.sin_addr.s_addr = INADDR_ANY;
    server.sin_port        = htons(3000); // Defined by USARSim Manual P.50
    if(0 > bind(portal_socket, (struct sockaddr*)&server, sizeof(server)))
    {
        perror("Bind failed\n");
        return 0;
    }
//While ?
    listen(portal_socket, MAX_COMMAND_CLIENTS);
    printf("USARGazebo:Waiting for connection\n");
    c = sizeof(struct sockaddr_in);
    rbuf[rbuf_num].Init();
    while(1)
    {
        if(MAX_COMMAND_CLIENTS > rbuf_num)
        {
            if((rbuf[rbuf_num].socket = accept(portal_socket
                            , (struct sockaddr *)&client, (socklen_t*)&c)))
            {
                printf("USARGazebo:Connection accepted\n");
SCK_TXT(rbuf[rbuf_num].socket, (char*)"USARGazebo:Message from accept_loop:assigned a new handler\n");
                if(0 > pthread_create(&thread_hnd, NULL
                      , usarcommand_client_connection, (void*)&rbuf[rbuf_num]))
                {
                    perror("Could not create thread\n");
                    exit(-1);
                }
                rbuf_num++;
                //pthread_join(thread_hnd, NULL);
            }
            else
            {
                perror("Accept fail\n");
                exit(-1);
            }
        }
    }
    return 0;
}

//#######################################################################
//  Image Server
//#######################################################################

struct USARimage
{
    int socket;
    char model_name[100], own_name[100], topic_root[100];
};

#define MAX_IMAGE_CLIENTS      10 // Depend on USARsim Manual ?
USARimage       ibuf[MAX_IMAGE_CLIENTS];
int             ibuf_num=0;

//#######################################################################
// SaveAsPPM saves an image for debug

void    SaveAsPPM(char* filename, ConstImageStampedPtr &_msg)
{
  unsigned char* ip = (unsigned char*)_msg->image().data().c_str();
  FILE* fp = fopen(filename, "wt");
  fprintf(fp, "P3\n%d %d\n255\n", _msg->image().width(),_msg->image().height());
  for(int i=0;i<_msg->image().width()*_msg->image().height();i++)
    fprintf(fp, "%3d %3d %3d\n", ip[i*3], ip[i*3+1], ip[i*3+2]);
  fclose(fp);
}

//#######################################################################
// Function is called everytime a message is received.
// 

void imageserver_callback(ConstImageStampedPtr& _msg)
{
  static int filenumber=0;
  char filename[100];
  if(filenumber>10)
    return;
  sprintf(filename, "./tmp%02d.PPM", (filenumber++)%10);
  SaveAsPPM(filename, _msg);
}

void *imageserver_client_connection(void* ibuf_ptr)
{
    USARimage*  ip = (USARimage*)ibuf_ptr;
SCK_TXT(ip->socket, (char*)"USARGazeboImageServer:Message from imageserver_child_connection\n");
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();
    gazebo::transport::SubscriberPtr sub 
        = node->Subscribe("~/pioneer3at_with_sensors/chassis/r_camera/image"
          , imageserver_callback);
    while(1)
        gazebo::common::Time::MSleep(10);
    return 0;
}

//      this->usarsimSub = this->node->Subscribe("~/usarsim",
//                                             &USARGazebo::OnUsarSim, this);

void *imageserver_accept_loop(void* dummy)
{
    pthread_t          thread_hnd;
    struct sockaddr_in server, client;
    int                c, portal_socket;
    portal_socket = socket(AF_INET, SOCK_STREAM, 0);
    if(-1 == portal_socket)
    {
        perror("Could not create socket\n");
        exit(-1);
    }
    server.sin_family      = AF_INET;
    server.sin_addr.s_addr = INADDR_ANY;
    server.sin_port        = htons(5003); // Defined by USARSim Manual ?
    if(0 > bind(portal_socket, (struct sockaddr*)&server, sizeof(server)))
    {
        perror("Bind failed\n");
        return 0;
    }
//While ?
    listen(portal_socket, MAX_IMAGE_CLIENTS);
    printf("USARGazeboImageServer:Waiting for connection\n");
    c = sizeof(struct sockaddr_in);
    while(1)
    {
        if(MAX_IMAGE_CLIENTS > ibuf_num)
        {
            if((ibuf[ibuf_num].socket = accept(portal_socket
                            , (struct sockaddr *)&client, (socklen_t*)&c)))
            {
                printf("USARGazeboImageServer:Connection accepted\n");
SCK_TXT(ibuf[ibuf_num].socket, (char*)"USARGazeboImageServer:Message from imageserver_accept_loop:assigned a new handler\n");
                if(0 > pthread_create(&thread_hnd, NULL
                              , imageserver_client_connection
                              , (void*)&ibuf[ibuf_num]))
                {
                    perror("Could not create image server thread\n");
                    exit(-1);
                }
                ibuf_num++;
                //pthread_join(thread_hnd, NULL);
            }
            else
            {
                perror("ImageServer Accept fail\n");
                exit(-1);
            }
        }
    }
    return 0;
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
//      this->connections.clear();
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
        if(pthread_create(&thread_hnd, NULL
            , usarcommand_accept_loop, (void*)NULL) < 0)
        {
            perror("could not create thread\n");
        }
        if(pthread_create(&imageserver_thread_hnd, NULL
            , imageserver_accept_loop, (void*)NULL) < 0)
        {
            perror("imageserver could not create thread\n");
        }
    }

/*
    ///////////////////////////////////////////////
    /// \brief Receive a command from a usarsim program
    public: void OnUsarSim(ConstGzStringPtr &_msg)
    {
      if (_msg->data() == "init")
      {
        // Create the message
        msgs::Factory msg;
        // Model file to load
        msg.set_sdf_filename("model://pioneer3at_with_sensors");
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
        physics::ModelPtr model = this->world->GetModel("pioneer3at_with_sensors");
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

    /////////////////////////////////////////////
    /// \brief Called every PreRender event. See the Load function.
    private: void Update()
    {
       // Create a new transport node
        transport::NodePtr node(new transport::Node());
       // Initialize the node with the world name
        node->Init();
       // Create a publisher on the ~/factory topic
        transport::PublisherPtr factoryPub
            = node->Advertise<msgs::Factory>("~/factory");
       // Create the message
        msgs::Factory msg;
       // Model file to load
       // msg.set_sdf_filename("model://cylinder");
       // msg.set_edit_name("cylinder1");
       // Pose to initialize the model to
        msgs::Set(msg.mutable_pose()
           , math::Pose(math::Vector3(1, -2, 0), math::Quaternion(0, 0, 0)));
       // Send the message
        factoryPub->Publish(msg);
    }
*/

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
  };
  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(USARGazebo)
}
