#bin/bash

cd nav/rosws/fastlio_localization

source devel/setup.bash
roslaunch fast_lio_localization localization_mid360.launch &

sleep 3

cd ..
cd livox_ros_driver2

source devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch &

sleep 3

cd ~/teleoperation 
python pos_server.py &
