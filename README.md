# simple_capture
simple capture for csi camera v4l2

g++ -std=c++17 -Wall -Wextra -g simple_csi_capture.cpp -o simple_csi_capture
./simple_csi_capture /dev/video0 ./csi_frames 640 480 NV12 10
