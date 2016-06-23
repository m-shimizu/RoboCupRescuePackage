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

// Japanese comments will be removed soon, soory. M.Shimizu

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

double DEG_1 = -45;
double DEG_2 = +45;

namespace gazebo
{
class MobileBasePlugin : public ModelPlugin
{
  transport::NodePtr node;
  physics::ModelPtr  model;
  common::Time       simTime;

  transport::SubscriberPtr velSub;
  transport::SubscriberPtr statsSub;
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

  /// Wheel speed and gain
  double THETA[30];
  double gain;
 
  /// Distance between wheels on the same axis (Determined from SDF)
  double wheelSeparation;

  /// Radius of the wheels (Determined from SDF)
  double wheelRadius;

  /// Arm flag
  int arm_flg;

  public:
  MobileBasePlugin(void)
  {
    arm_flg         = 0;
    wheelRadius     = 0.2;
    wheelSeparation = 1;
    for(int i = 0; i < 30; i++)
      THETA[i] = 0;
  }

  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
  {
    // physics::WorldPtr world = physics::get_world("default");
    this->model = _model;
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(this->model->GetWorld()->GetName());
    if(this->LoadParams(_sdf))
    {
      this->velSub = this->node->Subscribe(
      std::string("~/") + this->model->GetName() + std::string("/vel_cmd"),
      &MobileBasePlugin::OnVelMsg, this);
      this->updateConnection
        = event::Events::ConnectWorldUpdateBegin(
                  boost::bind(&MobileBasePlugin::OnUpdate, this));
    }
  }
 
  bool LoadParams(sdf::ElementPtr _sdf)
  {
    if(!_sdf->HasElement("gain"))
    {
        gzerr << "param [gain] not found\n";
        return false;
    }
    else
        this->gain = _sdf->Get<double>("gain");
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
    return true;
  }

  /////////////////////////////////////////////////
  void OnVelMsg(ConstPosePtr &_msg)
  {
    // gzmsg << "cmd_vel: " << msg->position().x() << ", "
    //       <<msgs::Convert(msg->orientation()).GetAsEuler().z<<std::endl;
    double vel_lin = _msg->position().x() / this->wheelRadius;
    double vel_rot = -1 * msgs::Convert(_msg->orientation()).GetAsEuler().z
                     * (this->wheelSeparation / this->wheelRadius);
    set_velocity(vel_lin - vel_rot, vel_lin + vel_rot);
  }

  void set_velocity(double  vr, double  vl)
  {
    THETA[1] = vr;
    THETA[2] = vr;
    THETA[3] = vr;
    THETA[4] = vr;
    THETA[13] = vr;
    THETA[14] = vr;
    THETA[15] = vr;
    THETA[19] = vr;
    THETA[20] = vr;
    THETA[21] = vr;
    THETA[25] = vr;
    THETA[26] = vr;

    THETA[5] = vl;
    THETA[6] = vl;
    THETA[7] = vl;
    THETA[8] = vl;
    THETA[16] = vl;
    THETA[17] = vl;
    THETA[18] = vl;
    THETA[22] = vl;
    THETA[23] = vl;
    THETA[24] = vl;
    THETA[27] = vl;
    THETA[28] = vl;
  }

  /////////////////////////////////////////////////
/* UNDER CONSTRUCTION
  void OnArmMsg(ConstPosePtr &_msg)
  {
    // gzmsg << "cmd_vel: " << msg->position().x() << ", "
    //       <<msgs::Convert(msg->orientation()).GetAsEuler().z<<std::endl;
  
    double vel_lin = _msg->position().x() / this->wheelRadius;
    double vel_rot = -1 * msgs::Convert(_msg->orientation()).GetAsEuler().z
                     * (this->wheelSeparation / this->wheelRadius);

    set_velocity(vel_lin - vel_rot, vel_lin + vel_rot);
  }

  void set_arm_angle(void)
  {
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
  }
*/
 
  /////////////////////////////////////////////////
  void MoveWheel(void)
  {
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
  }

  void MoveArm(void)
  {
    float right_front_deg=0, left_front_deg=0,
          right_rear_deg=0,  left_rear_deg=0;
    math::Angle angle1 = hinge9->GetAngle(0);   // get current arm angles
    math::Angle angle2 = hinge10->GetAngle(0);
    math::Angle angle3 = hinge11->GetAngle(0);   
    math::Angle angle4 = hinge12->GetAngle(0);
printf("right_front=%6.3f right_rear=%6.3f left_front=%6.3f left_rear=%6.3f \r ", angle1.Degree(), angle2.Degree(), angle3.Degree(), angle4.Degree());
    right_front_deg = angle1.Degree();
    left_front_deg  = angle3.Degree();
    right_rear_deg  = angle2.Degree();
    left_rear_deg   = angle4.Degree();
//    if(1 == arm_flg)
    {
      dev_RF = right_front_deg-DEG_1;
      THETA[9]= dev_RF*(-1)*Gp + dev_RF_sum*(-1)*Gi;
      dev_RF_sum += dev_RF;
      dev_LF = left_front_deg-DEG_1;    
      THETA[11]= dev_LF*(-1)*Gp + dev_LF_sum*(-1)*Gi; 
      dev_LF_sum += dev_LF;
      dev_RR = right_rear_deg-DEG_2;
      THETA[10]= dev_RR*(-1)*Gp + dev_RR_sum*(-1)*Gi;
      dev_RR_sum += dev_RR;
      dev_LR = left_rear_deg-DEG_2;    
      THETA[12]= dev_LR*(-1)*Gp + dev_LR_sum*(-1)*Gi; 
      dev_LR_sum += dev_LR;
    }
    hinge9->SetVelocity(0, THETA[9]);
    hinge10->SetVelocity(0, THETA[10]);
    hinge11->SetVelocity(0, THETA[11]);
    hinge12->SetVelocity(0, THETA[12]);
  }
 
  public:
  void OnUpdate()
  {
    MoveWheel();
    MoveArm();
  }
};

GZ_REGISTER_MODEL_PLUGIN(MobileBasePlugin)
}
