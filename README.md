# BlueRov_ROS2_Gstreamer
This a C++ ROS2 wrapper that streams images from bluerov2 and publishes it as a compressed images getting images from the blueOS ip address
The Gstreamer is C based light weight image streamer platform that enables fast and efficient streaming of images. 

This code is used to stream images directly from the raspberry pi running BlueOS on Bluerov2 Heavy.
[Gstreamer](https://gstreamer.freedesktop.org/)


## Dependencies
- OpenCV 4.x used
- Gstreamer 1.x

## Installations
```sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio```

## How to Install
```
mkdir -p /bluerov_driver_ws
cd bluerov-driver_ws && mkdir src
cd src
git clone https://github.com/Danny024/BlueRov_ROS2_Gstreamer.git
cd ~/bluerov_driver_ws
colcon build
```

## System Used
Ubuntu 22.04
ROS 2 Humble

## How to Use 
```
cd ~/bluerov_driver_ws
source install/setup.bash
ros2 launch bluerov_driver video.launch.py
```




