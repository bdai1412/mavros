# How to use offboard_node and for simulation.
> ref the link: https://dev.px4.io/en/simulation/gazebo.html
+ run:
    + cd Firmware
    + make posix_sitl_default gazebo
+ run:
```
HERE=$HOME/src/Firmware
source ${HERE}/Tools/setup_gazebo.bash ${HERE} ${HERE}/build/posix_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${HERE}
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:${HERE}/Tools/sitl_gazebo
```
+ then run:
    + roslaunch px4 posix_sitl.launch

    + roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"

+ run: 
    + rosrun mavros offboard_node