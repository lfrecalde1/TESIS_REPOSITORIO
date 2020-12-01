#!/bin/bash

sudo apt-get install openssh-server

sudo systemctl enable ssh

sudo service ssh restart

sudo dpkg-reconfigure openssh-server

ls -l /dev/ttyACM*

sudo usermod -a -G dialout robot

<<<<<<< HEAD
sudo usermod -a -G dialout robot
=======
<<<<<<< HEAD
sudo usermod -a -G dialout robot
=======
sudo usermod -a -G dialout osboxes
>>>>>>> 09bdd9d308db06bed19216644baf11414d97c25a
>>>>>>> d27498bca004882d7c283b355bd3adec1fcc6087
