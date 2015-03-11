# RoboCupRescuePackage
A package adding USARSim interface to Gazebo

## How to setup
        $ cd ~
        $ git clone https://github.com/m-shimizu/RoboCupRescuePackage/
        $ cd ~/RoboCupRescuePackage
        $ source ./.bashrc.USARGaz
        $ mkdir build
        $ cd build
        $ make

## How to use
        * At Terminal1
        $ cd ~/RoboCupRescuePackage
        $ source ./.bashrc.USARGaz
        $ ./USRAGaz.run
        
        * At Terminal2
        $ telnet localhost 3000
        (and you can see spawing a robot into a gazebo window)

## Status at 2015.3.10

### Current Function 
* Waiting for a socket connection at port 3000
* When accepting a socket connection, spawn a robot into a gazebo

### Current Constructing Point
* Command recognizer

### Workable USARSim command
         -none

### Completed rate
    0---10---20---30---40---50---60---70---80---90---100 %
    |++++++++++++++|
