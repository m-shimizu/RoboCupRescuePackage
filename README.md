# RoboCupRescuePackage for RoboCup Resuce Virtual Robot League
A package adding USARSim interface to Gazebo


## Requirements
* Hardware : core i-7(CLK 3GHz), Mem 8GB, nVidia Geforce GTX780Ti
* Software :  
  Recommend : Ubuntu 14, Gazebo 5 (If your ROS version is indigo)  
 (Maybe OK  : Ubuntu 12, Gazebo 4 (If your ROS version is hydro)  

### Software Environment Installation Tips  
This package requires Gazebo 5.  
And this package does not required ROS.  
Then if any ROS version has not been installed, only you have to type following command:  

        $ sudo apt-get install libgazebo5 libgazebo5-dev gazebo5
        $ sudo apt-get install protobuf-compiler  

If already you are using ROS indigo, Gazebo 2 may be installed instead of Gazebo 5.  
In the case, at first, you have to remove the installed Gazebo 2, and then, you can install Gazebo 5.

        $ sudo apt-get purge gazebo2*  
        $ sudo apt-get autoremove  
        $ sudo apt-get install libgazebo5 libgazebo5-dev gazebo5  
        $ sudo apt-get install protobuf-compiler  

#### Trouble Shooting for an error in installation of software environment   
        In Gazebo5 installation, I often encountered an error "broken dependency error".  
        If you got the error, try following steps:  

        1st step: Start with a clean installed Ubuntu 14.  
             (no tips, just do it by an ordinally method)  

        2nd step: ROS indigo installation, if you need ROS indigo.  
        $ sudo apt-get install ros-indigo-desktop  

        3rd step: Gazebo5 installation with running over "broken dependency error"  
        $ sudo dpkg --configure -a  
        $ sudo apt-get install -f  
        $ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list'  
        $ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -  
        $ sudo apt-get update  
        $ sudo apt-get install libgazebo5 libgazebo5-dev  
        $ sudo apt-get install gazebo5  
        $ sudo apt-get install protobuf-compiler  

        done.  
        
        If you are already using Gazebo4 and ROS hydro by installing drcsim-hydro package, maybe you need libgazebo4-dev package instead of libgazebo5-dev.  
        $ sudo apt-get install libgazebo4-dev  

        
## How to setup  
        $ cd ~  
        $ git clone https://github.com/m-shimizu/RoboCupRescuePackage/  
        $ cd ~/RoboCupRescuePackage  
        $ source ./.bashrc.USARGazebo  
        $ mkdir build  
        $ cd build  
        $ cmake ../  
        $ make  


## How to use  
        Start four terminal windows  

        * At Terminal1  
        $ cd ~/RoboCupRescuePackage  
        $ source ./.bashrc.USARGazebo  
        $ gazebo USARGazebo.world  
        
        * At Terminal2  
        $ telnet localhost 3000  
        (and you can use some commands : GETSTARTPOSES, INIT, DRIVE)  
        Ex.  
        (1) Getting Usable Start Location  
            GETSTARTPOSES  
        (2) Spawn a robot at the coordinate (5 , 10 , 0.3)  
            INIT {ClassName pioneer3at_with_sensors}{Name Robot1}{Location 5,10,0.3}  
        (3) Move the robot  
            DRIVE {RIGHT 1.0}{LEFT 1.0}  
        
        * At Terminal3  
        $ telnet localhost 5003  
        (and you can get a frame raw camara image data, after you send "OK")  
        Ex.(This command returns a frame of image in USARSim style. You will see so many charactors on your screen, but it's a correct response from this command.)  
            OK  
        
        * At Terminal4  
        $ cd ~/RoboCupRescuePackage  
        $ source ./.bashrc.USARGazebo  
        $ cd build  
        $ ./robot_teleop pioneer3at_with_sensors 2  
        (and you can control the robot with the keyboard)  

        "robot_teleop" is a program to control a robot which have a plugin "SkidSteerDrivePlugin" through topic "/vel_cmd".  
        The 1st argument is the model name. Please check it in the model name tree in the left sidebar of the gazebo window. If you see a different name like "pioneer3at_with_sensors_0", replace the 1st argument with it.   
        The 2nd argument is a plugin type number defined in this program. Do not change.  
        To stop this program, push control key and C key on your keyboard at same time.  
        
        * Shutdown process of Gazebo needs 15~20 sec. You should make an interval before starting Gazebo again.  

## Describing XML tags for USARGazebo in a world file.
    * Location and orientation of start point
      <StartPose_1>X Y Z Roll Pitch Yaw</StartPose_1> 
      <StartPose_2>X Y Z Roll Pitch Yaw</StartPose_2> 
      <StartPose_3>X Y Z Roll Pitch Yaw</StartPose_3> 
      <StartPose_4>X Y Z Roll Pitch Yaw</StartPose_4> 
    * Turning Ground Truth on
      <GroundTruth>true</GroundTruth>

## Developping Status at 2016.6.24  

### Current Function  
* Waiting for socket connections at port 3000 and port 5003  
* You can spawn a robot into a gazebo with "INIT" command like next line:  
    INIT {ClassName pioneer3at_with_sensors}{Name Robo_A}{Location 1,-2,0}{Rotation 0,0,0}  
  - Spawning multi robots requires 1 sec interval between each robot spawning.
* You can drive the robot with "DRIVE" command like next line:  
    DRIVE {Right 0.5}{Left 0.5}  
* You can move flipper arms of a crawler robot with "MULTIDRIVE" command like next line:  
    MULTIDRIVE {FRFlipper 1.57}{FLFlipper 1.57}{RRFlipper 1.57}{RLFlipper 1.57}  
* You can get a long landscaped jpeg image consisted from 4 robot cameras with following steps. Each robot camera image's width x height is 640 x 480. Camera images are located side-by-side and first spawned robot's camera image is located at left end of a jpeg image. If a robot has two cameras, currently you can see right camera's image.  
    1. Make a tcp socket connection at port 5003  
    2. Send "OK"  
    3. Get one frame raw image data from camera  
* You can get following informations from a robot:  
    1. STA (information is limited)
    2. SEN (Type : RangeScanner , GPS , INS)
    3. NFA (Return of a command : GETSTARTPOSES )
* You can set 9 start pose informations in USARGazebo.world as option parameters of USARGazebo plugin.  

### Robots and equipments
* pioneer3at_with_sensors series  
    - Classname : pioneer3at_with_sensors , pioneer3at_with_sensors_r , pioneer3at_with_sensors_b , pioneer3at_with_sensors_g , pioneer3at_with_sensors_y  
    - Equipments : one cameras and one range sensor.  

* pioneer2dx_with_sensors series  
    - Classname : pioneer2dx_with_sensors_r , pioneer2dx_with_sensors_b , pioneer2dx_with_sensors_g , pioneer2dx_with_sensors_y  
    - Equipments : one cameras and one range sensor.  

* turtlebot_with_sensors series  
    - Classname : turtlebot_with_sensors_r , turtlebot_with_sensors_b , turtlebot_with_sensors_g , turtlebot_with_sensors_y  
    - Equipments : one cameras and one range sensor.  

* crawler_robot series  
    - Classname : crawler_robot_r , crawler_robot_b , crawler_robot_g , crawler_robot_y  
    - Equipments : one cameras, one range sensor and four flipper arms.  
    - Flipper arms : You can control the angle of a flipper arm with MULTIDRIVE command.  
       MULTIDRIVE {FRFlipper 1.57}{FLFlipper 1.57}{RRFlipper 1.57}{RLFlipper 1.57}  

### Sample Client Software  
* A sample client software was prepared to show fundamental usage of the RoboCupRescuePackage.  
* See https://github.com/m-shimizu/USARSimSampleClient  
* What you can see with the USARSimSampleClient:  
  - Spawning 4 robots  
  - 4 robot's camera images transfered in jpeg format are shown on rviz  

### Current Constructing Point  
* Increasing robots which you can use.

### Workable USARSim command  
* USARSim commands  
    - INIT (limited)  
    - GETCONF (limited)  
    - GETGEO (limited)  
    - DRIVE (limited)  
    - MULTIDRIVE (only for controlling flipper arms)  
    - GETSTARTPOSES (It can return effective start point parameters. It can not read parameters from map but from world file as plugin options)  
* Image Server  
    - OK (Jpeg only(if you need raw image, please request))  

### USARSim commands which are underconstruction or scheduled  
    - SET  
    - MISPGK  

### Completed rate (sponsered by RoboCup Foundation)  
    0---10---20---30---40---50---60---70---80---90---100 %  
    |+++++++++++++++++++++++++++++++++++++++++++|  

### Acknowledgement  
    Original maps produced by Stephen Balakirsky and the National Institute of Standards and Technology (NIST).  

### Change log  
    * 24/ 6/2016 : crawler_robot was added.
    * 22/ 6/2016 : pioneer2dx and turtlebot were added.
    * 20/ 5/2016 : Jpeg camera image enabled.
    * 17/ 2/2016 : Add output of sensors.
    * 17/ 6/2015 : GETSTARTPOSES can return start point parameters. GETSTARTPOSES reads parameters from a world file which call the USARGazebo plugin as plugin options.  
    * 22/ 4/2015 : Adding INIT and DRIVE command and a function for sending back camera image.  
                   Those have limitation on it's function.  
    * 12/ 3/2015 : Adding a limited ImageServer in USARGazebo.cc  
                   Now sending image data on TCP/IP is not realized,  
                   but you can see image files.  
    * 11/ 3/2015 : Adding a model which have 2 cameras, 1 hokuyo.  
                   The model's name is "pioneer3at_with_sensors".  
    * 11/ 3/2015 : Changed filename  
