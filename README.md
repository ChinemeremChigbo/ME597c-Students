# ME597 - Autonomous Mobile Robots - Setup

## Install pre-requisites of the course for x86_64 systems
If you have not installed Ubuntu 22.04 yet, please do that first.
Download the script (or if you installed ```git```, clone this repository with ```git clone https://github.com/ME597c/ME597c-Students.git```, then change the branch to ```setup``` with ```git checkout setup```)

Run the script (it needs sudo privilege, so you will need to type in your password when prompted):
```
sh setup_me597.sh
```
that will take care of installing everything you need for this course, including setting up some environmental variables.

## Install pre-requisites of the course for ARM64 systems
If you are using MacOS with apple silicon, the script above will not work. Please follow this blog [How to Set Up VMware, Ubuntu 22, ROS2, and Gazebo on Arm64 (like Apple Silicon or Jetson)](https://medium.com/@MinghaoNing/how-to-set-up-vmware-ubuntu-22-ros2-and-gazebo-on-arm64-like-apple-silicon-or-jetson-5bb4db6ff297) to set up your VM.
```

## Check your installation
Once the script has finished the installation, you can quickly check the performance of your system with the Gazebo simulation. Open a terminal and run:
```
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```
You should see something like this:

![image](https://github.com/user-attachments/assets/a46bb1e6-b3d1-4a85-9c09-79a4210eee09)

At the bottom of the Gazebo window, you will see the ```Real Time Factor```, if this number is consistently below 0.5 (assuming you are not running anything else heavy on your VM or on your computer), this means that your simulation will likely be quite slow, and you may want to consider utilizing one of the alternative systems proposed (you may still use your system for code development and connecting with the real robot, just the testing in simulation might be slow).


## To check the latency of the topics in TurtleBot4s
NOTE: This part may be needed when you will be using the physical robot to check the latency in the communications.

Use the Latency check script like this:

```
./latency_check.py topic msgType
# for example for scan topic
./latency_check.py /scan LaserScan 
```


