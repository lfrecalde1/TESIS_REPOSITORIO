#!/bin/bash

sudo apt-get install openssh-server

sudo systemctl enable ssh

sudo service ssh restart

sudo dpkg-reconfigure openssh-server

ls -l /dev/ttyACM*

sudo usermod -a -G dialout robot
