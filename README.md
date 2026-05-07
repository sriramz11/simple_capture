# simple_capture
simple capture for csi camera v4l2

g++ -std=c++17 -Wall -Wextra -g simple_csi_capture.cpp -o simple_csi_capture
./simple_csi_capture /dev/video0 ./csi_frames 640 480 NV12 10


CXX = g++

CXXFLAGS = -std=c++20 -Wall -Wextra -O2
OPENCV = $(shell pkg-config --cflags --libs opencv4)

all:
	$(CXX) $(CXXFLAGS) main.cpp -o dashcam $(OPENCV)

clean:
	rm -f dashcam
	rm -rf events
