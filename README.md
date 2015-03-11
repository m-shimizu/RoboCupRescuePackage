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
