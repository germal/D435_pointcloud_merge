#!/bin/bash
# Installs the librealsense SDK onto Jetson TX/Nano
# Link to official instructions on github
# https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md

if [ $(id -u) != "0" ]; then
echo "You must be the superuser to run this script" >&2
exit 1
fi

# Register the server's public key
sudo apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

# Add server to list of repositories
# This displays some errors, but works nontheless
sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main" -u

# Installation of SDK
sudo apt-get -y install librealsense2-utils
sudo apt-get -y install librealsense2-dev

# With dev package installed, you can compile an application with librealsense using g++ -std=c++11 filename.cpp -lrealsense2 or an IDE of your choice. To get started with RealSense using CMake check out librealsense/examples/cmake

# Run realsense-viewer to see if everything is set up OK.
echo "Now type realsense-viewer in the terminal to check everything has been installed correctly" >&2
