#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <stdio.h>
#include <gazebo/math/gzmath.hh>

#include <termios.h>
#include <iostream>

#define DEG00 0
#define DEG45 45
#define DEG90 90
#define DEG_45 -45
#define DEG_90 -90
#define Gp 0.05
#define Gi 0.00 //0.00005

int arm_deg = 0;
int arm_deg_2 = 0;
int arm_flg = 0;
float dev_RF = 0;
float dev_LF = 0;
float dev_RR = 0;
float dev_LR = 0;
double dev_RF_sum = 0;
double dev_LF_sum = 0;
double dev_RR_sum = 0;
double dev_LR_sum = 0;

double DEG_1 = 0;
double DEG_2 = 0;

namespace gazebo
{
class MobileBasePlugin : public ModelPlugin
{
public:
    void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
       // physics::WorldPtr world = physics::get_world("default");

        this->model = _parent;

        hinge1 = this->model->GetJoint("right_front");
  hinge2 = this->model->GetJoint("right_center1");
  hinge3 = this->model->GetJoint("right_center2");
        hinge4 = this->model->GetJoint("right_rear");

        hinge5 = this->model->GetJoint("left_front");
  hinge6 = this->model->GetJoint("left_center1");
  hinge7 = this->model->GetJoint("left_center2");
        hinge8 = this->model->GetJoint("left_rear");

        hinge9 = this->model->GetJoint("right_front_arm");
        hinge10 = this->model->GetJoint("right_rear_arm");
        hinge11 = this->model->GetJoint("left_front_arm");
        hinge12 = this->model->GetJoint("left_rear_arm");

  hinge13 = this->model->GetJoint("right_front_arm_wheel_1");
  hinge14 = this->model->GetJoint("right_front_arm_wheel_2");
  hinge15 = this->model->GetJoint("right_front_arm_wheel_3");

  hinge16 = this->model->GetJoint("left_front_arm_wheel_1");
  hinge17 = this->model->GetJoint("left_front_arm_wheel_2");
  hinge18 = this->model->GetJoint("left_front_arm_wheel_3");

  hinge19 = this->model->GetJoint("right_rear_arm_wheel_1");
  hinge20 = this->model->GetJoint("right_rear_arm_wheel_2");
  hinge21 = this->model->GetJoint("right_rear_arm_wheel_3");

  hinge22 = this->model->GetJoint("left_rear_arm_wheel_1");
  hinge23 = this->model->GetJoint("left_rear_arm_wheel_2");
  hinge24 = this->model->GetJoint("left_rear_arm_wheel_3");

  hinge25 = this->model->GetJoint("right_sub2");
  hinge26 = this->model->GetJoint("right_sub3");

  hinge27 = this->model->GetJoint("left_sub2");
  hinge28 = this->model->GetJoint("left_sub3");

        
        for (int i=0; i< 30; i++)
            THETA[i] = 0;

        if (this->LoadParams(_sdf))
        {
            this->updateConnection
            = event::Events::ConnectWorldUpdateBegin(
                  boost::bind(&MobileBasePlugin::OnUpdate, this));
        }
    }

 
    bool LoadParams(sdf::ElementPtr _sdf)
    {
        if (!_sdf->HasElement("gain"))
        {
            gzerr << "param [gain] not found\n";
            return false;
        }
        else
            this->gain = _sdf->Get<double>("gain");
        return true;
    }
 
int  doslike_kbhit(void)
{
  struct termios  oldt, newt;
  int  ch;
  int  oldf;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(BRKINT | ISTRIP | IXON);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
  ch = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
  return 0;
}

int  doslike_getch(void)
{
  static struct termios  oldt, newt;
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(BRKINT | ISTRIP | IXON);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK | ECHONL);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  int c = getchar();
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  return c;
}


void  check_key_command(void)
{

  if(doslike_kbhit())
  {
    int cmd = doslike_getch();
    switch(cmd)
    {
        case 'w':
            THETA[1] = 10;
            THETA[2] = 10;
            THETA[3] = 10;
            THETA[4] = 10;
            THETA[5] = 10;
            THETA[6] = 10;
            THETA[7] = 10;
      THETA[8] = 10;
      THETA[13] = 10;
      THETA[14] = 10;
      THETA[15] = 10;
      THETA[16] = 10;
      THETA[17] = 10;
      THETA[18] = 10;
      THETA[19] = 10;
      THETA[20] = 10;
      THETA[21] = 10;
      THETA[22] = 10;
      THETA[23] = 10;
      THETA[24] = 10;
            THETA[25] = 10;
            THETA[26] = 10;
            THETA[27] = 10;
            THETA[28] = 10;
            break;     

        case 'z':
            THETA[1] = -10;
            THETA[2] = -10;
            THETA[3] = -10;
            THETA[4] = -10;
            THETA[5] = -10;
            THETA[6] = -10;
            THETA[7] = -10;
      THETA[8] = -10;
      THETA[13] = -10;
      THETA[14] = -10;
      THETA[15] = -10;
      THETA[16] = -10;
      THETA[17] = -10;
      THETA[18] = -10;
      THETA[19] = -10;
      THETA[20] = -10;
      THETA[21] = -10;
      THETA[22] = -10;
      THETA[23] = -10;
      THETA[24] = -10;
            THETA[25] = -10;
            THETA[26] = -10;
            THETA[27] = -10;
            THETA[28] = -10;
            break;    

        case 'a':
            THETA[1] = 10;
            THETA[2] = 10;
            THETA[3] = 10;
            THETA[4] = 10;
            THETA[13] = 10;
            THETA[14] = 10;
            THETA[15] = 10;
            THETA[19] = 10;
            THETA[20] = 10;
            THETA[21] = 10;
            THETA[25] = 10;
            THETA[26] = 10;

            THETA[5] = -10;
            THETA[6] = -10;
            THETA[7] = -10;
      THETA[8] = -10;
            THETA[16] = -10;
            THETA[17] = -10;
            THETA[18] = -10;
            THETA[22] = -10;
            THETA[23] = -10;
            THETA[24] = -10;
            THETA[27] = -10;
            THETA[28] = -10;
            break;   

        case 'd':
            THETA[1] = -10;
            THETA[2] = -10;
            THETA[3] = -10;
            THETA[4] = -10;
            THETA[13] = -10;
            THETA[14] = -10;
            THETA[15] = -10;
            THETA[19] = -10;
            THETA[20] = -10;
            THETA[21] = -10;
            THETA[25] = -10;
            THETA[26] = -10;

            THETA[5] = 10;
            THETA[6] = 10;
            THETA[7] = 10;
      THETA[8] = 10;
            THETA[16] = 10;
            THETA[17] = 10;
            THETA[18] = 10;
            THETA[22] = 10;
            THETA[23] = 10;
            THETA[24] = 10;
            THETA[27] = 10;
            THETA[28] = 10;
            break;   

        case 's':
            THETA[1] = 0;
            THETA[2] = 0;
            THETA[3] = 0;
            THETA[4] = 0;
            THETA[5] = 0;
            THETA[6] = 0;
            THETA[7] = 0;
      THETA[8] = 0;
            THETA[13] = 0;
            THETA[14] = 0;
            THETA[15] = 0;
            THETA[19] = 0;
            THETA[20] = 0;
            THETA[21] = 0;
            THETA[16] = 0;
            THETA[17] = 0;
            THETA[18] = 0;
            THETA[22] = 0;
            THETA[23] = 0;
            THETA[24] = 0;
            THETA[25] = 0;
            THETA[26] = 0;
            THETA[27] = 0;
            THETA[28] = 0;
            break;  

        case 'y':
    arm_flg = 1;
    DEG_1 -= 1;
    dev_RF_sum = 0;
    dev_LF_sum = 0;
    dev_RR_sum = 0;
    dev_LR_sum = 0;
            break; 

        case 'u':
    arm_flg = 1;
    DEG_1 = 0;
    dev_RF_sum = 0;
    dev_LF_sum = 0;
    dev_RR_sum = 0;
    dev_LR_sum = 0;
            break;

        case 'i':
    arm_flg = 1;
    DEG_1 += 1;
    dev_RF_sum = 0;
    dev_LF_sum = 0;
    dev_RR_sum = 0;
    dev_LR_sum = 0;
      break;

        case 'h':
    arm_flg = 1;
    DEG_2 += 1;
    dev_RF_sum = 0;
    dev_LF_sum = 0;
    dev_RR_sum = 0;
    dev_LR_sum = 0;
            break; 

        case 'j':
    arm_flg = 1;
    DEG_2 = 0;
    dev_RF_sum = 0;
    dev_LF_sum = 0;
    dev_RR_sum = 0;
    dev_LR_sum = 0;
            break;
        
        case 'k':
    arm_flg = 1;
    DEG_2 -= 1;
    dev_RF_sum = 0;
    dev_LF_sum = 0;
    dev_RR_sum = 0;
    dev_LR_sum = 0;
      break;

        default:
            std::cout << "Input w,a,s,d,y,u,i,h,j,k" << std::endl;
    }
  }
}
 
 
  void moveJoint()
  {
    float right_front_deg=0,left_front_deg=0,right_rear_deg=0,left_rear_deg=0;
    math::Angle angle1 = hinge9->GetAngle(0);
    math::Angle angle2 = hinge10->GetAngle(0);
    math::Angle angle3 = hinge11->GetAngle(0);   
    math::Angle angle4 = hinge12->GetAngle(0);
printf("right_front=%6.3f right_rear=%6.3f left_front=%6.3f left_rear=%6.3f \r ", angle1.Degree(), angle2.Degree(), angle3.Degree(), angle4.Degree());

    right_front_deg=angle1.Degree();
    left_front_deg=angle3.Degree();
    right_rear_deg=angle2.Degree();
    left_rear_deg=angle4.Degree();

    hinge1->SetVelocity(0, THETA[1]);
    hinge2->SetVelocity(0, THETA[2]);
    hinge3->SetVelocity(0, THETA[3]);
    hinge4->SetVelocity(0, THETA[4]);
    hinge5->SetVelocity(0, THETA[5]);
    hinge6->SetVelocity(0, THETA[6]);
    hinge7->SetVelocity(0, THETA[7]);
    hinge8->SetVelocity(0, THETA[8]);
    hinge13->SetVelocity(0, THETA[13]);
    hinge14->SetVelocity(0, THETA[14]);
    hinge15->SetVelocity(0, THETA[15]);
    hinge16->SetVelocity(0, THETA[16]);
    hinge17->SetVelocity(0, THETA[17]);
    hinge18->SetVelocity(0, THETA[18]);
    hinge19->SetVelocity(0, THETA[19]);
    hinge20->SetVelocity(0, THETA[20]);
    hinge21->SetVelocity(0, THETA[21]);
    hinge22->SetVelocity(0, THETA[22]);
    hinge23->SetVelocity(0, THETA[23]);
    hinge24->SetVelocity(0, THETA[24]);
    hinge21->SetVelocity(0, THETA[25]);
    hinge22->SetVelocity(0, THETA[26]);
    hinge23->SetVelocity(0, THETA[27]);
    hinge24->SetVelocity(0, THETA[28]);

    if(arm_flg == 0)
    {
      hinge9->SetVelocity(0, THETA[9]);
      hinge10->SetVelocity(0, THETA[10]);
      hinge11->SetVelocity(0, THETA[11]);
      hinge12->SetVelocity(0, THETA[12]);
    }
    else if(arm_flg == 1)
    {
      dev_RF = right_front_deg-DEG_1;
      THETA[9]= dev_RF*(-1)*Gp + dev_RF_sum*(-1)*Gi;
      hinge9->SetVelocity(0, THETA[9]);
      dev_RF_sum += dev_RF;
      
      dev_LF = left_front_deg-DEG_1;    
      THETA[11]= dev_LF*(-1)*Gp + dev_LF_sum*(-1)*Gi; 
      hinge11->SetVelocity(0, THETA[11]);
      dev_LF_sum += dev_LF;

      dev_RR = right_rear_deg-DEG_2;
      THETA[10]= dev_RR*(-1)*Gp + dev_RR_sum*(-1)*Gi;
      hinge10->SetVelocity(0, THETA[10]);
      dev_RR_sum += dev_RR;
      
      dev_LR = left_rear_deg-DEG_2;    
      THETA[12]= dev_LR*(-1)*Gp + dev_LR_sum*(-1)*Gi; 
      hinge12->SetVelocity(0, THETA[12]);
      dev_LR_sum += dev_LR;
    }
  }
 
  public:
  void OnUpdate()
  {
    check_key_command();
    moveJoint();
  }

  physics::ModelPtr model;
 
  transport::NodePtr node;
  transport::SubscriberPtr statsSub;
  common::Time simTime;
 
  event::ConnectionPtr updateConnection;
  physics::JointPtr hinge1;
  physics::JointPtr hinge2;
  physics::JointPtr hinge3;
  physics::JointPtr hinge4;
  physics::JointPtr hinge5;
  physics::JointPtr hinge6;
  physics::JointPtr hinge7;
  physics::JointPtr hinge8;
  physics::JointPtr hinge9;
  physics::JointPtr hinge10;
  physics::JointPtr hinge11;
  physics::JointPtr hinge12;
  physics::JointPtr hinge13;
  physics::JointPtr hinge14;
  physics::JointPtr hinge15;
  physics::JointPtr hinge16;
  physics::JointPtr hinge17;
  physics::JointPtr hinge18;
  physics::JointPtr hinge19;
  physics::JointPtr hinge20;
  physics::JointPtr hinge21;
  physics::JointPtr hinge22;
  physics::JointPtr hinge23;
  physics::JointPtr hinge24;
  physics::JointPtr hinge25;
  physics::JointPtr hinge26;
  physics::JointPtr hinge27;
  physics::JointPtr hinge28;

  //sensors::RaySensorPtr laser;
  physics::LinkPtr  sensor;
  double THETA[30];
  double gain;
};
 
GZ_REGISTER_MODEL_PLUGIN(MobileBasePlugin)
}
