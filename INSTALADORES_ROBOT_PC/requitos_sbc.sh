#!/bin/bash 
echo "ACTULIZAR LINUX" 
sudo apt-get update
sudo apt-get upgrade

echo "DESCARGAR PAQUETES NECESARIOS OPENCV"
sudo apt-get install build-essential cmake unzip pkg-config
sudo apt-get install libjpeg-dev libpng-dev libtiff-dev
sudo apt-get install libjasper-dev
sudo add-apt-repository "deb http://security.ubuntu.com/ubuntu xenial-security main"
sudo apt update
sudo apt install libjasper1 libjasper-dev
sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
sudo apt-get install libxvidcore-dev libx264-dev
sudo apt-get install libgtk-3-dev
sudo apt-get install libatlas-base-dev gfortran
sudo apt-get install python3.6-dev

echo "INSTALAR GIT"
sudo apt-get update
sudo apt-get install git-core
git --version

echo "INSTALAR GEDIT"
sudo apt install gedit

echo "INSTALAR CMAKE"
sudo apt remove --purge cmake
hash -r
sudo snap install cmake --classic
cmake --version

echo "DESCARGAR EL REPOSITORIO DE OPENCV"
cd ~
wget -O opencv.zip https://github.com/opencv/opencv/archive/3.4.4.zip
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/3.4.4.zip
unzip opencv.zip 
unzip opencv_contrib.zip
mv opencv-3.4.4 opencv
mv opencv_contrib-3.4.4 opencv_contrib

echo "CONFIGURACION PARA CREACION DE ENTORNOS VIRTUALES"
wget https://bootstrap.pypa.io/get-pip.py
sudo python3 get-pip.py
sudo pip install virtualenv virtualenvwrapper
sudo rm -rf ~/get-pip.py ~/.cache/pip

echo -e "\n# virtualenv and virtualenvwrapper" >> ~/.bashrc
echo "export WORKON_HOME=$HOME/.virtualenvs" >> ~/.bashrc
echo "export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3" >> ~/.bashrc
echo "source /usr/local/bin/virtualenvwrapper.sh" >> ~/.bashrc


source ~/.bashrc


