# RoboCupRescuePackage for RoboCup Resuce Virtual Robot League
A package adding USARSim interface to Gazebo

## Requirements
* Hardware : core i-7(CLK 3GHz), Mem 8GB, nVidia Geforce GTX680Ti        
* Software :   
  Recommend : Ubuntu 14, Gazebo 5, ROS indigo  
  Maybe OK  : Ubuntu 12, Gazebo 4, ROS hydro  

### Software Environment Installation Tips  
        I often encountered that Gazebo5 installation 
        was stopped by an  "broken dependency error".  
        If you meet the same error, try following steps:  

        1st step: Start with a clean Ubuntu 14 installation 
             (no tips, just do it by an ordinally method)  

        2nd step: ROS indigo installation  
        $ sudo apt-get install ros-indigo-desktop-full  

        3rd step: Gazebo5 installation with running over "broken dependency error"
        $ sudo dpkg --configure -a  
        $ sudo apt-get install -f  
        $ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list'
        $ wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
        $ sudo apt-get update
        $ sudo apt-get install libgazebo5 libgazebo5-dev  
        $ sudo apt-get install gazebo5  

        4th step: Other elements installation
        $ sudo apt-get install protobuf-compiler

        done.  
        
        If you are already using Gazebo4 and ROS hydro by installing drcsim-hydro package, maybe you need libgazebo4-dev package.
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
        $ gazebo USRAGazebo.world
        
        * At Terminal2
        $ telnet localhost 3000
        (and you can use some commands : GETSTARTPOSES, INIT, DRIVE)
        
        * At Terminal3  
        $ telnet localhost 5003  
        (and you can get a frame raw camara image data, after you send "OK")  
        
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

## Status at 2015.6.17

### Current Function 
* Waiting for socket connections at port 3000 and port 5003
* You can spawn a robot into a gazebo with "INIT" command like next line:  
    INIT {ClassName pioneer3at_with_sensors}{Name Robo_A}{Location 1,-2,0}{Rotation 0,0,0}  
* You can drive the robot with "DRIVE" command like next line:  
    DRIVE {Right 0.5}{Left 0.5}  
* You can get a frame image of camera embeded on the robot with following steps. But now you can not choose camera.  
    1. Make a tcp socket connection at port 5003  
    2. Send "OK"  
    3. Get one frame raw image data from camera  

### Current Constructing Point  
* Writing codes for sending back sensor data.  
    ** Sample world of this project has a robot model which have two cameras and a range sensor.  

### Workable USARSim command
* USARSim commands  
    - INIT (limited)  
    - DRIVE (limited)  
    - GETSTARTPOSES (It can return effective start point parameters. It can not read parameters from map but from world file as plugin options)  
* Image Server  
    - OK (Raw Image Data only)  

### Completed rate (sponsered by RoboCup Foundation)
    0---10---20---30---40---50---60---70---80---90---100 %
    |+++++++++++++++++++++++++++++++++++++++|

### Change log
    * 17/ 6/2015 : GETSTARTPOSES can return start point parameters. GETSTARTPOSES reads parameters from a world file which call the USARGazebo plugin as plugin options.
    * 22/ 4/2015 : Adding INIT and DRIVE command and a function for sending back camera image.   
                   Those have limitation on it's function.
    * 12/ 3/2015 : Adding a limited ImageServer in USARGazebo.cc
                   Now sending image data on TCP/IP is not realized,
                   but you can see image files.
    * 11/ 3/2015 : Adding a model which have 2 cameras, 1 hokuyo.
                   The model's name is "pioneer3at_with_sensors".
    * 11/ 3/2015 : Changed filename
