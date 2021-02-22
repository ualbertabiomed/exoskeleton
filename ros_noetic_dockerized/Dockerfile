FROM ubuntu:20.04
LABEL version="0.1"
LABEL description="barebone install of ros noetic on ubuntu 20.04"

# Upgrade installed packages
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get upgrade -y

# Install dependencies
RUN apt-get update \
    && apt-get install -y gnupg2 curl lsb-release libxext-dev libxrender-dev libxtst-dev

# Install ROS Noetic
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | apt-key add -
RUN apt-get update && apt-get install -y ros-noetic-desktop-full

# Setup environment
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN /bin/bash -c "source ~/.bashrc"

# Install ROS dependencies
RUN apt-get update \
    && apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

# Initialize rosdep
RUN rosdep init
RUN rosdep fix-permissions
RUN rosdep update

# Default command
WORKDIR /usr/src/
CMD ["/bin/bash"]
