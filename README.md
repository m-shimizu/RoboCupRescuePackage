# RoboCupRescuePackage for RoboCup Resuce Virtual Robot League
A package adding USARSim interface to Gazebo

## Requirements
* Hardware : core i-7(CLK 3GHz), Mem 8GB, nVidia Geforce GTX680Ti        
* Software : 
  Recommend : Ubuntu 14, Gazebo 5, ROS indigo
  Maybe OK  : Ubuntu 12, Gazebo 4, ROS hydro

### Installation tips  
        I often stopped installation by "broken dependency error".  
        My shortest method is here:  

        1st step: Ubuntu 14 installation(just do it by an ordinally method)  

        2st step: ROS indigo installation  
        $ sudo apt-get install ros-indigo-desktop-full  

        3nd step: Gazebo5 installation with running over "broken dependency error"
        $ sudo dpkg --configure -a  
        $ sudo apt-get install -f  
        $ sudo apt-get install libgazebo5 libgazebo5-dev  
        $ sudo apt-get install gazebo5  

        4th step: Other elements installation
        $ sudo apt-get install protobuf-compiler

        done.  

## How to setup
        $ cd ~
        $ git clone https://github.com/m-shimizu/RoboCupRescuePackage/
        $ cd ~/RoboCupRescuePackage
        $ source ./.bashrc.USARGazebo
        $ mkdir build
        $ cd build
        $ make

## How to use
        * At Terminal1
        $ cd ~/RoboCupRescuePackage
        $ source ./.bashrc.USARGazebo
        $ ./USRAGazebo.run
        
        * At Terminal2
        $ telnet localhost 3000
        (and you can see spawing a robot into a gazebo window)

        * Temporally method to control robot
        (After a robot spawned by abave method, open another terminal)
        $ cd ~/RoboCupRescuePackage
        $ source ./.bashrc.USARGazebo
        $ cd build
        $ ./robot_teleop pioneer3at_with_sensors 2
        "robot_teleop" is a program to control a robot which have a plugin "SkidSteerDrivePlugin" through topic "/vel_cmd".  
        The 1st argument is the model name. Please check it in the model name tree in the left sidebar of the gazebo window. If you see a different name like "pioneer3at_with_sensors_0", replace the 1st argument with it.  
        The 2nd argument is a plugin type number defined in this program. Do not change.  
        To stop this program, push control key and C key on your keyboard at same time.  

## Status at 2015.3.11

### Current Function 
* Waiting for a socket connection at port 3000
* When accepting a socket connection, spawn a robot into a gazebo

### Current Constructing Point
* Find a method to transfer camera image and other sensor data.
    ** Make a robot model which have cameras and a range sensor.
* Command recognizer(stopping)

### Workable USARSim command
         -none

### Completed rate (sponsered by RoboCup Foundation)
    0---10---20---30---40---50---60---70---80---90---100 %
    |++++++++++++++|

### Change log
    * 11/ 3/2015 : Adding a model which have 2 cameras, 1 hokuyo.
                   The model's name is "pioneer3at_with_sensors".
    * 11/ 3/2015 : Changed filename
