/*
 * Copyright 2012 Open Source Robotics Foundation
 * Copyright 2013 Dereck Wonnacott
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/math/gzmath.hh>

#include <termios.h>
#include <iostream>

enum DRIVE_TYPE
{
  RT_DiffarentialDrive = 1,
  RT_SkidsteerDrive    = 2
};

//char  DefaultRobotName[][20] = {"Default", "pioneer2dx", "pioneer3at"};

void  publish_vel_cmd(gazebo::transport::PublisherPtr pub, float speed = 0, float turn = 0, int RobotType = 0)
{
  switch(RobotType)
  {
    case 1: //turn = turn;
      // Pioneer 2DX(Skidsteer); Turn > 0 then robot turn clock wise
      break;
    case 2: turn = -turn;
      // Pioneer 3AT(Diffarential); Turn < 0 then robot turn clock wise
      break;
    default: speed = turn = 0;
      break;
  }
  gazebo::math::Pose pose(speed, 0, 0, 0, 0, turn);
  gazebo::msgs::Pose msg;
  gazebo::msgs::Set(&msg, pose);
  pub->Publish(msg);
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

void  disp_usage_control_and_status(float speed, float turn)
{
  static int  loop_counter = 0;
  if(0 == (loop_counter++ % 10))
  {
    printf("\n              [w]:speed up to front\n");
    printf("[a]:left turn [s]:stop        [d]:right turn\n");
    printf("              [x]:speed up to back\n");
		printf("\nAnd with a game controller, you can use the right analog stick to move the robot\n");
		printf("Current programmed game controllers are:\n  BUFFALO BSGP1601\n");
  }
  printf("  Speed : %8.3f , Turn : %8.3f\r", speed, turn);
}

#define _MAX(X,Y)  ((X > Y)?X:Y)
#define _MIN(X,Y)  ((X < Y)?X:Y)

#include <linux/joystick.h>
#define BUTTON_DATA_MAX 20
#define STICK_DATA_MAX 4

#define Kjx	( 2.0/30000.0)
#define Kjy (-2.0/30000.0)

void  check_joystick(gazebo::transport::PublisherPtr pub, int RobotType
                     , float& speed, float& turn)
{
  // See http://wlog.flatlib.jp/item/1682
  // and https://www.kernel.org/doc/Documentation/input/joystick-api.txt
//  unsigned char ButtonData[BUTTON_DATA_MAX];
//  signed int    StickData[STICK_DATA_MAX];
  struct js_event jse;
  int fd = open("/dev/input/js0", O_RDONLY | O_NONBLOCK);
  if(0 > fd)
  {
//    fprintf(stderr, "Joystick open error\n");
    return;
  }
  while(read(fd, &jse, sizeof(jse)) > 0)
  {
//  printf("%02X : %5d, %5d\n", jse.type, jse.number, jse.value);
    switch(jse.type & 0x7f)
    {
      case JS_EVENT_BUTTON:
        if(jse.number < BUTTON_DATA_MAX)
        {
//          printf("BUTTON[%d]:%-5d\n", jse.number, jse.value);
  //        ButtonData[jse.number] = jse.value;
        }
        break;
      case JS_EVENT_AXIS:
        if(jse.number < STICK_DATA_MAX)
        {
          switch(jse.number)
          {
            case 3: speed = jse.value * Kjy; break;
            case 2: turn  = jse.value * Kjx; break;
          }
  /* [BUFFALO BSGP1601]
     Left Analog Stick       Right Analog Stick         Hat Swtiches
           [1]                     [3]                      [5]
          -32767                 -32767                    -32767
 [0]-32767  +  +32767    [2]-32767  +  +32767    [4]-32767   +   +32767
          +32767                 +32767                    +32767
  */
//          printf("STICK[%d]:%-5d\n", jse.number, jse.value);
  //      StickData[jse.number] = jse.value;
        }
        break;
    }
  }
  close(fd);
  if(speed < 0)
    turn *= -1.0; // For natural backward behavior 
  publish_vel_cmd(pub, speed, turn, RobotType);
  disp_usage_control_and_status(speed, turn);
}

void  check_key_command(gazebo::transport::PublisherPtr pub, int RobotType
                        , float& speed, float& turn)
{
  if(doslike_kbhit())
  {
    int cmd = doslike_getch();
    switch(cmd)
    {
      case 's': turn  = 0;
      case 'S': speed = 0;
          break;
      case 'W': turn   = 0;
      case 'w': speed += 0.2;
          break;
      case 'X': turn   = 0;
      case 'x': speed -= 0.2;
          break;
      case 'A': speed = 0;
      case 'a': turn -= 0.2;
          turn = _MAX(turn, -3.14);
          break;
      case 'D': speed = 0;
      case 'd': turn += 0.2;
          turn = _MIN(turn, 3.14);
          break;
    }
    publish_vel_cmd(pub, speed, turn, RobotType);
    disp_usage_control_and_status(speed, turn);
  }
}

void  disp_usage_option(char* arg)
{
  printf("%s robotname robottype{1:diff, 2:skid} [worldname]\n", arg);
  printf("Example1: %s pioneer2dx 1\n", arg);
  printf("Example2: %s pioneer3at 2\n", arg);
  printf("\n   [worldname] is an optional argument and it is defined in *.world file.\n");
  printf("Example3: %s pioneer3at 2 test_world\n", arg);
}

int main(int argc, char* argv[])
{
  char  worldname[50];
	float speed = 0, turn = 0;
  if(3 != argc && 4 != argc)
  {
    disp_usage_option(argv[0]);
    return -1;
  }
  else
  {
    
    if(3 == argc)
      strcpy(worldname, "default");
    else if(4 == argc)
      strcpy(worldname, argv[3]);
    printf("Robot name = %s\nRobot Type = %d\nWorld name = %s\n", argv[1], atoi(argv[2]), worldname);
  }
  int  RobotType = atoi(argv[2]);
  char  TopicName[100];
  gazebo::transport::init();
  gazebo::transport::run();
  gazebo::transport::NodePtr  node(new gazebo::transport::Node());
  node->Init(worldname);
  sprintf(TopicName, "~/%s/vel_cmd", argv[1]);
printf("%s\n", TopicName);
  gazebo::transport::PublisherPtr pub = node->Advertise<gazebo::msgs::Pose>(TopicName);
  pub->WaitForConnection();
  disp_usage_control_and_status(0, 0);
  for(;1;)
  {
    gazebo::common::Time::MSleep(100);
    check_key_command(pub, RobotType, speed, turn);
    check_joystick(pub, RobotType, speed, turn);
  }
  return 0;
}
