FROM dorowu/ubuntu-desktop-lxde-vnc:latest

WORKDIR /root/project

# Install dependencies
RUN sudo apt update && \
	apt install -y build-essential && \
	apt install -y cmake && \
	apt install -y git && \
	apt install -y unzip && \
	apt install -y libpcl-dev && \
	apt install -y pkg-config
