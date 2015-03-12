# RoboCupRescuePackage for RoboCup Resuce Virtual Robot League
A package adding USARSim interface to Gazebo

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

       * Tempollary method to control robot
        (After a robot spawned by abave method, open another terminal)
        $ cd ~/RoboCupRescuePackage
        $ source ./.bashrc.USARGazebo
        $ cd build
        $ ./robot_teleop pioneer3at_with_sensors 2
        robot_teleop is a program to control a robot which have a plugin "SkidSteerDrivePlugin" through topic "/vel_cmd".  
        The 1st argument is the model name. Please check it in the model name tree in the left sidebar of the gazebo window.  
        The 2nd argument is kind of plugin. Do not change.  
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
