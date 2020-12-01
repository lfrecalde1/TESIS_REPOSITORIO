#!/bin/bash
# virtualenv and virtualenvwrapper
export WORKON_HOME=$HOME/.virtualenvs
export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3
source /usr/local/bin/virtualenvwrapper.sh

cd ~

mkdir -p ~/catkin_ws/src

cd ~/catkin_ws/

workon cv

catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3

source devel/setup.bash

echo "source $HOME/catkin_ws/devel/setup.bash" >> ~/.bashrc

source ~/.bashrc


