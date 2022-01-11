FROM dorowu/ubuntu-desktop-lxde-vnc:latest

WORKDIR /root/project

# Install dependencies
RUN sudo apt update && \
	apt install -y build-essential && \
	apt install -y cmake && \
	apt install -y git && \
	apt install -y unzip && \
	apt install -y libpcl-dev && \
	apt install -y pkg-config && \
    apt install -y g++ && \
    apt install -y wget && \

# Download and unpack sources
wget -O opencv.zip https://github.com/opencv/opencv/archive/4.x.zip && \
wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.x.zip && \
unzip opencv.zip && \
unzip opencv_contrib.zip && \
# Create build directory and switch into it
mkdir -p build && cd build && \
# Configure
cmake -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-4.x/modules ../opencv-4.x && \
# Build
cmake --build . -j5 --verbose
