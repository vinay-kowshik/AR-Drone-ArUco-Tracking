# INTRALOG-ar_drone
[AR Drone 2.0](https://github.com/vinay-kowshik/AR-Drone-ArUco-Tracking/wiki/Useful-Information#parrot-ar-drone-20---power-edition) is a quadcopter designed by Parrot Inc.
This repository contains code for ArUco detection through AR Drone 2.0. 

But before that, there are some pre-requisities to be installed before using the code in this repo.

   * OpenCV & OpenCV Contrib Modules
   * ROS (Robot Operating System)
   * AR Drone Autonomy

## Steps to use this script provided in the repository
**Step - 1**: Installation of Pre-requisite softwares
This [Wiki](../../wiki/Home) takes you through the Installation Procedure - for all relevant SDKs on Ubuntu 16.04 LTS.and also with some basic information required prior to this project.

**Step - 2**: Create Catkin workspace in the Home directory. This [Page](https://github.com/vinay-kowshik/AR-Drone-ArUco-Tracking/wiki/Installation-Procedure#ros-workspace-creation) will take you through the creation of the workspace. Now you can see a folder created in Home Directory called `catkin_ws`.

**Step - 3**: Create a Catkin package under `catkin_ws/src` folder. This [Page](https://github.com/vinay-kowshik/AR-Drone-ArUco-Tracking/wiki/Installation-Procedure#ros-package-creation) will take you through the instruction to create the package. After going through this step successfully, a folder named `ros_opencv` will be created in `catkin_ws/src` folder.

**Step - 4**: Download the files from the repository. It contains the ArUco tracking C++ code and all relevant files for running the code. Command to clone the repository is given below:
```
cd ~/catkin_ws/src
git clone https://github.com/vinay-kowshik/AR-Drone-ArUco-Tracking.git
```
A folder named `INTRALOG-ar_drone` will created in `catkin_ws/src` Directory

**Step - 5**: Replace the **CMakeLists.txt** (which was created after Catkin Package creation) with the file available in the downloaded git repo `INTRALOG-ar_drone` inside `catkin_ws/src` folder. The file can be found in this [Link](https://github.com/vinay-kowshik/AR-Drone-ArUco-Tracking/blob/master/src/CMakeLists.txt)

**Step - 7**: Now open 3 terminals and execute following commands in each of the terminals

   * Terminal - 1: Run `roscore` to launch ROS
   * Terminal - 2: Run `rosrun ardrone_autonomy ardrone_driver` to activate AR Drone 2.0 Driver
   * Terminal - 3: Run the following commands
```
cd ~/INTRALOG-ar_drone/src
g++ Drone_tracking.cpp -o Drone_Tracking $(pkg-config --cflags --libs opencv) -I/opt/ros/kinetic/include -L/opt/ros/kinetic/lib -Wl,-rpath,/opt/ros/kinetic/lib -lroscpp -lrosconsole -lrostime -lboost_system -lcv_bridge -limage_transport -lroscpp_serialization -ltf   #(This is to compile the C++ Program)
./Drone_Tracking  #(This is to run the compiled program)
```
