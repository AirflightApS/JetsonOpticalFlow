# JetsonOpticalFlow
This project aims to compute optical flow using a feature based tracking algorithm powered by OpenCV. The software is intended to be run on a Jetson Nano, connected by MAVLink to a PX4 Flight controller, enabling position control in indoor conditions without GPS. The camera is sampled through the gstreamer pipeline (nvarguscamerasrc), but the software can be modified to support ofther input sources.

## Hardware
- Jetson Nano 2/4GB
- CSI camera
