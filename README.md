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

## Current Funcion
* Waiting for a socket connection at port 3000
* When accepting a socket connection, spawn a robot into a gazebo

## Workable USARSim command (2015.3.10)
  -none
