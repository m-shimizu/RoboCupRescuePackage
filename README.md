# RoboCupRescuePackage for RoboCup Resuce Virtual Robot League
A package adding USARSim interface to Gazebo

## Requirements
* Hardware : core i-7(CLK 3GHz), Mem 8GB, nVidia Geforce GTX680Ti        
* Software :   
  Recommend : Ubuntu 14, Gazebo 5, ROS indigo  
  Maybe OK  : Ubuntu 12, Gazebo 4, ROS hydro  

### Software Environment Installation Tips  
        I often stopped Gazebo5 installation by "broken dependency error".  
        Followings are my installation memo:  

        1st step: Ubuntu 14 installation(no tips, just do it by an ordinally method)  

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
        * At Terminal1
        $ cd ~/RoboCupRescuePackage
        $ source ./.bashrc.USARGazebo
        $ gazebo USRAGazebo.world
        
        * At Terminal2
        $ telnet localhost 3000
        (and you can see spawing a robot into a gazebo window)
        
        * At Terminal3  
        $ telnet localhost 5003  
        (and you can see 10 image files in RoboCupRescuePackage directory)  
        (It's for checking of read image data from Gazebo image topic)  
        (Sending Image data on TCP/IP as USARSim Imageserver is now constructing)  
        
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
        
        * Shutdown process of Gazebo needs 15~20 sec. You should make an interval before starting Gazebo again.

## Status at 2015.3.12

### Current Function 
* Waiting for a socket connection at port 3000
* When accepting a socket connection, spawn a robot into a gazebo

### Current Constructing Point
* Find a method to transfer camera image and other sensor data.  
    ** Made a robot model which have cameras and a range sensor.  
    ** Making a prototype new ImageServer.  
* Command recognizer(restarting of development)

### Workable USARSim command
         -none

### Completed rate (sponsered by RoboCup Foundation)
    0---10---20---30---40---50---60---70---80---90---100 %
    |++++++++++++++++++++++++|

### Change log
    * 12/ 3/2015 : Adding a limited ImageServer in USARGazebo.cc
                   Now sending image data on TCP/IP is not realized,
                   but you can see image files.
    * 11/ 3/2015 : Adding a model which have 2 cameras, 1 hokuyo.
                   The model's name is "pioneer3at_with_sensors".
    * 11/ 3/2015 : Changed filename
