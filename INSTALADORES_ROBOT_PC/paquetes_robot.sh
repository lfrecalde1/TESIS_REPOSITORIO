#!/bin/bash
# virtualenv and virtualenvwrapper
export WORKON_HOME=$HOME/.virtualenvs
export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3
source /usr/local/bin/virtualenvwrapper.sh

cd ~

cd ~/catkin_ws/

cd ~/catkin_ws/src/

git clone https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver.git

git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git

git clone https://github.com/ROBOTIS-GIT/turtlebot3.git

cd ~/catkin_ws/src/turtlebot3

rm -r turtlebot3_description/ turtlebot3_teleop/ turtlebot3_navigation/ turtlebot3_slam/ turtlebot3_example/

sudo apt-get install ros-melodic-rosserial-python ros-melodic-tf

workon cv

cd

cd ~/catkin_ws/

catkin_make








