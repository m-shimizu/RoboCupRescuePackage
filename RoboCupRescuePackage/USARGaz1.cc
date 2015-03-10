#include <gazebo/math/Rand.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/rendering/rendering.hh>
#include <gazebo/gazebo.hh>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <pthread.h>

struct RD 
{
	char	Request;
	char	Done;
	void	Init(void) {Request = Done = 0;}
	RD(void) {Init();}
};
 
struct USARrobot
{
	RD      Msg, Spawn;
	char	model_name[100];
	void    Init(void) { Msg.Init(); Spawn.Init(); }
        int     socket;
	USARrobot(void) {Init();}
};


#define	MAX_ROBOTS	16 // Depend on USARsim Manual P.50
USARrobot	rbuf[MAX_ROBOTS];
int		rbuf_num=0;

void *child_connection(void* rbuf_ptr)
{
    USARrobot*  rp = (USARrobot*)rbuf_ptr;
char* message;
message = (char*)"Message from child_connection\n";
    write(rp->socket, message, strlen(message));
    while(1)
    {
        if(0 == rp->Spawn.Request)
        {
            rp->Spawn.Request = 1;
            rp->Msg.Request   = 1;
        }
    }
//    free(rp->socket);
    return 0;
}

void *accept_loop(void* dummy)
{
    pthread_t          thread_hnd;
    struct sockaddr_in server, client;
    int                c, portal_socket;
    portal_socket = socket(AF_INET, SOCK_STREAM, 0);
char* message;
    if(-1 == portal_socket)
    {
        perror("Could not create socket\n");
        exit -1;
    }
    server.sin_family      = AF_INET;
    server.sin_addr.s_addr = INADDR_ANY;
    server.sin_port        = htons(3000);
    if(0 > bind(portal_socket, (struct sockaddr*)&server, sizeof(server)))
    {
        perror("Bind failed\n");
        return 0;
    }
    listen(portal_socket, MAX_ROBOTS);
puts("Waiting for request\n");
    c = sizeof(struct sockaddr_in);
    rbuf[rbuf_num].Init();
    while(1)
    {
	if(MAX_ROBOTS > rbuf_num)
	{
            if((rbuf[rbuf_num].socket = accept(portal_socket
                            , (struct sockaddr *)&client, (socklen_t*)&c)))
            {
puts("Connection accepted\n");
message = (char*)"Message from accept_loop:assigned a new handler\n";
write(rbuf[rbuf_num].socket , message , strlen(message));
                if(0 > pthread_create(&thread_hnd, NULL
                              , child_connection, (void*)&rbuf[rbuf_num]))
                {
                    perror("Could not create thread\n");
                    exit -1;
                }
                rbuf_num++;
                //pthread_join(thread_hnd, NULL);
            }
            else
            {
                perror("Accept fail\n");
                exit -1;
            }
        }
    }
    return 0;
}

namespace gazebo
{
  class USARGaz : public SystemPlugin
  {
    /////////////////////////////////////////////
    /// \brief Destructor
    public: virtual ~USARGaz()
    {
      this->connections.clear();
//      if (this->userCam)
//        this->userCam->EnableSaveFrame(false);
//      this->userCam.reset();
    }

    /////////////////////////////////////////////
    /// \brief Called after the plugin has been constructed.
    public: void Load(int /*_argc*/, char ** /*_argv*/)
    {
      this->connections.push_back(
          event::Events::ConnectPreRender(
            boost::bind(&USARGaz::Update, this)));
    }

    pthread_t thread_hnd;
    /////////////////////////////////////////////
    // \brief Called once after Load
    private: void Init()
    {
        if(pthread_create(&thread_hnd, NULL, accept_loop, (void*)NULL) < 0)
        {
            perror("could not create thread");
        }
    }

    /////////////////////////////////////////////
    /// \brief Called every PreRender event. See the Load function.
    private: void Update()
    {
printf("-\n");
// ##################################################################
      if(0 < rbuf_num)
	if(1==rbuf[rbuf_num-1].Msg.Request)
	{
// it must be needed => if(1==rbuf[rbuf_num-1].Spawn.Request)
		rbuf[rbuf_num-1].Msg.Request = 0;
		rbuf[rbuf_num-1].Spawn.Done  = 1;
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
    //  msg.set_sdf_filename("model://cylinder");
printf("--\n");
      msg.set_sdf_filename("model://pioneer3at");
printf("---\n");
    //  msg.set_edit_name("cylinder1");
    //  msg.set_clone_model_name("cylinder1");
      msg.set_edit_name("cylinder1");
printf("----\n");
    // Pose to initialize the model to
printf("-----B\n");
      msgs::Set(msg.mutable_pose()
         , math::Pose(math::Vector3(1, -2, 0), math::Quaternion(0, 0, 0)));
printf("------\n");
    // Send the message
      factoryPub->Publish(msg);
printf("-------\n");
    }
// ##################################################################
//      if (!this->userCam)
//      {
        // Get a pointer to the active user camera
//        this->userCam = gui::get_active_camera();

        // Enable saving frames
//        this->userCam->EnableSaveFrame(true);

        // Specify the path to save frames into
//        this->userCam->SetSaveFramePathname("/tmp/gazebo_frames");
//      }

      // Get scene pointer
//      rendering::ScenePtr scene = rendering::get_scene();

      // Wait until the scene is initialized.
//      if (!scene || !scene->GetInitialized())
//        return;

      // Look for a specific visual by name.
//      if (scene->GetVisual("ground_plane"))
//        std::cout << "Has ground plane visual\n";
    }

    /// Pointer the user camera.
//    private: rendering::UserCameraPtr userCam;

    /// All the event connections.
    private: std::vector<event::ConnectionPtr> connections;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_SYSTEM_PLUGIN(USARGaz)
}
